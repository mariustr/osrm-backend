#include "extractor/guidance/graph_hopper.hpp"
#include "util/coordinate_calculation.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

GraphHopper::GraphHopper(const util::NodeBasedDynamicGraph &node_based_graph,
                         const IntersectionGenerator &intersection_generator)
    : node_based_graph(node_based_graph), intersection_generator(intersection_generator)
{
}

LengthLimitedCoordinateAccumulator::LengthLimitedCoordinateAccumulator(
    const extractor::guidance::CoordinateExtractor &coordinate_extractor, const double max_length)
    : coordinate_extractor(coordinate_extractor), max_length(max_length), accumulated_length(0)
{
}

bool LengthLimitedCoordinateAccumulator::terminate() { return accumulated_length >= max_length; }

// update the accumulator
void LengthLimitedCoordinateAccumulator::update(const NodeID from_node,
                                                const EdgeID via_edge,
                                                const NodeID to_node,
                                                const util::NodeBasedEdgeData &edge_data)
{
    const auto current_coordinates = coordinate_extractor.GetCoordinatesAlongRoad(
        from_node, via_edge, edge_data.reversed, to_node);

    const auto length = util::coordinate_calculation::getLength(
        coordinates, util::coordinate_calculation::haversineDistance);

    // in case we get too many coordinates, we limit them to our desired length
    if (length + accumulated_length > max_length)
        coordinate_extractor.TrimCoordinatesToLength(current_coordinates,
                                                     max_length - accumulated_length);

    coordinates.insert(coordinates.end(), current_coordinates.begin(), current_coordinates.end());

    accumulated_length += length;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
