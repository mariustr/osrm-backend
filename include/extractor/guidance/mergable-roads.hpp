#ifndef OSRM_EXTRACTOR_GUIDANCE_MERGEABLE_ROADS
#define OSRM_EXTRACTOR_GUIDANCE_MERGEABLE_ROADS

#include "extractor/guidance/intersection.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

/*
 * When it comes to merging roads, we need to find out if two ways actually represent the same road.
 * This check tries to identify roads which are the same road in opposite directions
 */
inline bool areSameRoad(const ConnectedRoad &lhs,
                        const ConnectedRoad &rhs,
                        const util::NodeBasedDynamicGraph &node_based_grap)
{
    // We first check for data compatibility between the road classes
    const auto &lhs_edge_data = node_based_grap.GetEdgeData(lhs.turn.eid);
    const auto &rhs_edge_data = node_based_grap.GetEdgeData(rhs.turn.eid);

    // this function checks whether a road could classify as a valid candidate for merging, based on
    // the edge data information alone. It does not consider the shape yet
    const auto describe_same_road = [&](const auto &lhs_edge_data, const auto &rhs_edge_data) {
        // to describe the same road, but in opposite directions (which is what we require for a
        // merge), the roads have to feature one reversed and one non-reversed edge
        if (lhs_edge_data.reversed == rhs_edge_data.reversed)
            return false;

        // The Roads need to offer the same name.
        // For merging, we are very strict. Usually we would check if the names are similar. We
        // might consider using `requiresNameAnncounced` here, but better be safe than sorry
        if (lhs_edge_data.name_id != rhs_edge_data.name_id || lhs_edge_data.name_id == EMPTY_NAMEID)
            return false;

        // The travel mode should be the same for both roads. If we were to merge different travel
        // modes, we would hide information/run the risk of loosing valid choices (e.g. short period
        // of pushing)
        if (lhs_edge_data.travel_mode != rhs_edge_data.travel_mode)
            return false;

        return lhs_edge_data.road_classification == rhs_edge_data.road_classification;
    };

    // to be the same road, represented in two OSM ways, we need obviously need the road data to
    // match  up. If this is not the case, we can stop right here
    if (!describe_same_road(lhs_edge_data, rhs_edge_data))
        return false;

    // all checks succeeded
    return true;
}


inline bool isM

/*
 * Segregated Roads often merge onto a single intersection.
 * While technically representing different roads, they are
 * often looked at as a single road.
 * Due to the merging, turn Angles seem off, wenn we compute them from the
 * initial positions.
 *
 *         b<b<b<b(1)<b<b<b
 * aaaaa-b
 *         b>b>b>b(2)>b>b>b
 *
 * Would be seen as a slight turn going fro a to (2). A Sharp turn going from
 * (1) to (2).
 *
 * In cases like these, we megre this segregated roads into a single road to
 * end up with a case like:
 *
 * aaaaa-bbbbbb
 *
 * for the turn representation.
 * Anything containing the first u-turn in a merge affects all other angles
 * and is handled separately from all others.
 */

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif
