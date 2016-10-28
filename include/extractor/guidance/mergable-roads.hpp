#ifndef OSRM_EXTRACTOR_GUIDANCE_MERGEABLE_ROADS
#define OSRM_EXTRACTOR_GUIDANCE_MERGEABLE_ROADS

#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/coordinate_extractor.hpp"
#include "extractor/guidance/intersection.hpp"
#include "extractor/query_node.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include "util/geojson_debug_logger.hpp"
#include "util/geojson_debug_policies.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// Forward declaration to be able to access the intersection_generator
class IntersectionGenerator;

/*
 * When it comes to merging roads, we need to find out if two ways actually represent the same road.
 * This check tries to identify roads which are the same road in opposite directions
 */
inline bool haveCompatibleRoadData(const util::NodeBasedEdgeData &lhs_edge_data,
                                   const util::NodeBasedEdgeData &rhs_edge_data)
{
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
}

inline bool connectAgain(const NodeID intersection_node,
                         const ConnectedRoad &lhs,
                         const ConnectedRoad &rhs,
                         const util::NodeBasedDynamicGraph &node_based_graph,
                         const IntersectionGenerator &intersection_generator)
{
    // compute the set of all intersection_nodes along the way of an edge, until it reaches a
    // location with the same name repeatet at least three times
    const auto findMeetUpCandidate = [&](const NameID searched_name, const ConnectedRoad &road) {
        auto current_node = intersection_node;
        auto current_eid = road.turn.eid;

        const auto has_requested_name = [&](const ConnectedRoad &road)
        {
            return name_id == node_based_graph.GetEdgeData(road.turn.eid).name_id;
        };

        // limit our search to at most 10 intersections
        for (std::size_t hop_count = 0; hope_count < 10; ++hop_count)
        {
            const auto next_intersection = intersection_generator(current_node, current_eid);
            const auto count = std::count_if(
                next_intersection.begin() + 1, next_intersection.end(), has_requested_name);

            if (count >= 2)
                return node_based_igraph.GetTarget(current_eid);
            else if (count == 0)
            {
                return SPECIAL_NODEID;
            }
            else
            {
                current_node = node_based_graph.GetTarget(current_eid);
                // skip over bridges/similar
                if (intersection.size() == 2)
                    current_eid = next_intersection[1].turn.eid;
                else
                    current_eid = std::find_if(
                        next_intersection.begin() + 1, next_intersection.end(), has_requested_name);
            }
        }

        return SPECIAL_NODEID;
    }
}

// Check if two roads go in the general same direction
inline bool haveSameDirection(const NodeID intersection_node,
                              const ConnectedRoad &lhs,
                              const ConnectedRoad &rhs,
                              const util::NodeBasedDynamicGraph &node_based_graph,
                              const IntersectionGenerator &intersection_generator,
                              const std::vector<QueryNode> &node_coordinates,
                              const CoordinateExtractor &coordinate_extractor)
{
    // Find a coordinate following a road that is far away
    const std::function<util::Coordinate(const ConnectedRoad &, const double)>
        findCoordinateFollowingRoad = [&](const ConnectedRoad &road, const double length) {
            const auto &road_edge_data = node_based_graph.GetEdgeData(rhs.turn.eid);
            const auto coordinates = coordinate_extractor.GetCoordinatesAlongRoad(
                intersection_node,
                road.turn.eid,
                road_edge_data.reversed,
                node_based_graph.GetTarget(lhs.turn.eid));
            const auto local_length = util::coordinate_calculation::getLength(
                coordinates, util::coordinate_calculation::haversineDistance);

            // small helper function to check for same name-ids
            const auto has_same_name = [&](const ConnectedRoad &next_road_candidate) {
                return node_based_graph.GetEdgeData(road.turn.eid).name_id ==
                       node_based_graph.GetEdgeData(next_road_candidate.turn.eid).name_id;
            };

            if (local_length >= length)
            {
                return coordinate_extractor.TrimCoordinatesToLength(std::move(coordinates), length)
                    .back();
            }
            else
            {
                // look at the next intersection
                const auto next_intersection =
                    intersection_generator.GetConnectedRoads(intersection_node, road.turn.eid);

                if (next_intersection.size() == 2)
                    return findCoordinateFollowingRoad(next_intersection[1], length - local_length);
                else
                {
                    BOOST_ASSERT(next_intersection.size() >= 1);
                    // and check if we can find a good way to continue on our current path
                    const auto count_same_name = std::count_if(std::begin(next_intersection) + 1,
                                                               std::end(next_intersection),
                                                               has_same_name);
                    // if there is no way of uniquely continuing on the road, stop
                    if (count_same_name != 1)
                        return coordinates.back();
                    const auto next_road = std::find_if(std::begin(next_intersection) + 1,
                                                        std::end(next_intersection),
                                                        has_same_name);
                    return findCoordinateFollowingRoad(*next_road, length - local_length);
                }
            }
        };

    const double assumed_lane_width = [&]() {
        const auto &lhs_edge_data = node_based_graph.GetEdgeData(lhs.turn.eid);
        const auto &rhs_edge_data = node_based_graph.GetEdgeData(rhs.turn.eid);

        return (std::max<std::uint8_t>(1, lhs_edge_data.road_classification.GetNumberOfLanes()) +
                std::max<std::uint8_t>(1, (rhs_edge_data.road_classification.GetNumberOfLanes()))) *
               3.25;
    }();

    const auto coordinate_to_left = findCoordinateFollowingRoad(lhs, 5 + 4 * assumed_lane_width);
    const auto coordinate_to_right = findCoordinateFollowingRoad(rhs, 5 + 4 * assumed_lane_width);
    const auto angle = util::coordinate_calculation::computeAngle(
        coordinate_to_left, node_coordinates[intersection_node], coordinate_to_right);

    return std::min(angle, 360 - angle) < 20;
}

// Try if two roads can be merged into a single one, since they represent the same road
inline bool canMergeRoad(const NodeID intersection_node,
                         const ConnectedRoad &lhs,
                         const ConnectedRoad &rhs,
                         const util::NodeBasedDynamicGraph &node_based_graph,
                         const IntersectionGenerator &intersection_generator,
                         const std::vector<QueryNode> &node_coordinates,
                         const CoordinateExtractor &coordinate_extractor)
{
    const auto &lhs_edge_data = node_based_graph.GetEdgeData(lhs.turn.eid);
    const auto &rhs_edge_data = node_based_graph.GetEdgeData(rhs.turn.eid);

    // roundabouts are special, simply don't hurt them. We might not want to bear the consequences
    if (lhs_edge_data.roundabout || rhs_edge_data.roundabout)
        return false;

    // mergable roads cannot hide a turn. We are not allowed to remove any of them
    if (lhs.entry_allowed && rhs.entry_allowed)
        return false;

    // and they need to describe the same road
    if (!haveCompatibleRoadData(lhs_edge_data, rhs_edge_data))
        return false;

    // finally check if two roads describe the same way
    if (!haveSameDirection(intersection_node,
                           lhs,
                           rhs,
                           node_based_graph,
                           intersection_generator,
                           node_coordinates,
                           coordinate_extractor))
        return false;

    // if all checks succeed, we are golden
    return angularDeviation(lhs.turn.angle, rhs.turn.angle) < 60;
}

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
