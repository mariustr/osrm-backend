#ifndef OSRM_EXTRACTOR_GUIDANCE_GRAPH_HOPPER
#define OSRM_EXTRACTOR_GUIDANCE_GRAPH_HOPPER

#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/intersection_generator.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "util/coordinate.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/optional.hpp>
#include <utility>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// forward declaration to allow interaction between the intersection generator and the graph hopper
class IntersectionGenerator;

/*
 * The graph hopper is a utility that lets you find certain intersections with a node based graph,
 * accumulating information along the way
 */
class GraphHopper
{
  public:
    GraphHopper(const util::NodeBasedDynamicGraph &node_based_graph,
                const IntersectionGenerator &intersection_generator);

    template <class accumulator_type>
    boost::optional<std::pair<NodeID, EdgeID>> TraverseRoad(NodeID starting_at_node_id,
                                                            EdgeID following_edge_id,
                                                            accumulator_type &accumulator);

  private:
    const util::NodeBasedDynamicGraph &node_based_graph;
    const IntersectionGenerator &intersection_generator;
};

// Accumulate all coordinates following a road until we
struct LengthLimitedCoordinateAccumulator
{
    LengthLimitedCoordinateAccumulator(
        const extractor::guidance::CoordinateExtractor &coordinate_extractor,
        const double max_length);

    // true if the path has traversed enough distance
    bool terminate();

    // update the accumulator
    void update(const NodeID from_node,
                const EdgeID via_edge,
                const NodeID to_node,
                const util::NodeBasedEdgeData &edge_data);

    const extractor::guidance::CoordinateExtractor &coordinate_extractor;
    const double max_length;
    double accumulated_length;
    std::vector<util::Coordinate> coordinates;
};

template <class accumulator_type>
boost::optional<std::pair<NodeID, EdgeID>> GraphHopper::TraverseRoad(NodeID current_node_id,
                                                                     EdgeID current_edge_id,
                                                                     accumulator_type &accumulator)
{
    // since graph hopping is used in many ways, we don't generate an adjusted intersection
    // (otherwise we could end up in infinite recursion if we call the graph hopper during the
    // adjustment itself). Relying only on `GetConnectedRoads` (which itself does no graph hopping),
    // we prevent this from happening.
    const auto stop_node_id = current_node_id;
    const auto segment_name_id = node_based_graph.GetEdgeData(current_edge_id).name_id;
    while (!accumulator.terminate())
    {
        accumulator.update(current_node_id,
                           current_edge_id,
                           node_based_graph.GetTarget(current_edge_id),
                           node_based_graph.GetEdgeData(current_edge_id));

        // find the next road to follow
        // small helper function to check for same name-ids
        const auto has_narrow_with_segment_name = [&](const ConnectedRoad &next_road_candidate) {
            return angularDeviation(next_road_candidate.turn.angle, STRAIGHT_ANGLE) <
                       NARROW_TURN_ANGLE &&
                   segment_name_id ==
                       node_based_graph.GetEdgeData(next_road_candidate.turn.eid).name_id;
        };

        // look at the next intersection
        const auto next_intersection =
            intersection_generator.GetConnectedRoads(current_node_id, current_edge_id);

        // don't follow u-turns or go past our initial intersection
        if (next_intersection.size() <= 1 ||
            node_based_graph.GetTarget(current_edge_id) == stop_node_id)
            return {};

        current_node_id = node_based_graph.GetTarget(current_edge_id);
        if (next_intersection.size() == 2)
        {
            current_edge_id = next_intersection[1].turn.eid;
        }
        else if (next_intersection.size() > 2)
        {
            // Follow the road if we find a single entry that has our name, only
            // and check if we can find a good way to continue on our current path
            const auto count_same_name = std::count_if(std::begin(next_intersection) + 1,
                                                       std::end(next_intersection),
                                                       has_narrow_with_segment_name);
            // if there is no way of uniquely continuing on the road, stop
            if (count_same_name != 1)
            {
                const auto straightmost = next_intersection.findClosestTurn(STRAIGHT_ANGLE);
                if (angularDeviation(straightmost->turn.angle, STRAIGHT_ANGLE) > NARROW_TURN_ANGLE)
                    return {};
                else
                    current_edge_id = straightmost->turn.eid;
            }
            else
            {
                const auto next_road = std::find_if(std::begin(next_intersection) + 1,
                                                    std::end(next_intersection),
                                                    has_narrow_with_segment_name);

                current_edge_id = next_road->turn.eid;
            }
        }
    }
    return {std::make_pair(current_node_id, current_edge_id)};
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif
