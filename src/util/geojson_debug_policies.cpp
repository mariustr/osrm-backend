#include "util/geojson_debug_policies.hpp"
#include "util/coordinate.hpp"

#include <algorithm>

namespace osrm
{
namespace util
{

namespace
{

struct CoordinateToJsonArray
{
    util::json::Array operator()(const util::Coordinate coordinate)
    {
        util::json::Array json_coordinate;
        json_coordinate.values.push_back(static_cast<double>(toFloating(coordinate.lon)));
        json_coordinate.values.push_back(static_cast<double>(toFloating(coordinate.lat)));
        return json_coordinate;
    }
};

struct NodeIdToCoordinate
{
    NodeIdToCoordinate(const std::vector<extractor::QueryNode> &node_coordinates)
        : node_coordinates(node_coordinates)
    {
    }

    const std::vector<extractor::QueryNode> &node_coordinates;

    util::json::Array operator()(const NodeID nid)
    {
        auto coordinate = node_coordinates[nid];
        CoordinateToJsonArray converter;
        return converter(coordinate);
    }
};

util::json::Object makeFeature(const std::string &type, util::json::Array coordinates)
{
    util::json::Object result;
    util::json::Object properties;
    result.values["type"] = "Feature";
    result.values["properties"] = properties;
    util::json::Object geometry;
    geometry.values["type"] = type;
    geometry.values["properties"] = properties;
    geometry.values["coordinates"] = coordinates;
    result.values["geometry"] = geometry;

    return result;
}

util::json::Array makeJsonArray(const std::vector<util::Coordinate> &input_coordinates)
{
    util::json::Array coordinates;
    std::transform(input_coordinates.begin(),
                   input_coordinates.end(),
                   std::back_inserter(coordinates.values),
                   CoordinateToJsonArray());
    return coordinates;
}

} // namespace

//----------------------------------------------------------------
NodeIdVectorToLineString::NodeIdVectorToLineString(
    const std::vector<extractor::QueryNode> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}

// converts a vector of node ids into a linestring geojson feature
util::json::Object NodeIdVectorToLineString::operator()(const std::vector<NodeID> &node_ids) const
{
    util::json::Array coordinates;
    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   NodeIdToCoordinate(node_coordinates));

    return makeFeature("LineString", coordinates);
}

//----------------------------------------------------------------
NodeIdVectorToMultiPoint::NodeIdVectorToMultiPoint(
    const std::vector<extractor::QueryNode> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}
util::json::Object NodeIdVectorToMultiPoint::operator()(const std::vector<NodeID> &node_ids) const
{
    util::json::Array coordinates;
    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   NodeIdToCoordinate(node_coordinates));

    return makeFeature("MultiPoint", coordinates);
}

//----------------------------------------------------------------
util::json::Object CoordinateVectorToMultiPoint::
operator()(const std::vector<util::Coordinate> &input_coordinates) const
{
    const auto coordinates = makeJsonArray(input_coordinates);
    return makeFeature("MultiPoint", coordinates);
}

//----------------------------------------------------------------
util::json::Object CoordinateVectorToLineString::
operator()(const std::vector<util::Coordinate> &input_coordinates) const
{
    const auto coordinates = makeJsonArray(input_coordinates);
    return makeFeature("LineString", coordinates);
}

} /* namespace util */
} /* namespace osrm */
