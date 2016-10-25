#include "util/geojson_debug_logger.hpp"

namespace osrm
{
namespace util
{
// make sure to define our local template variables
template <class geojson_conversion_policy, LoggingScenario scenario>
bool GeojsonLogger<geojson_conversion_policy, scenario>::first;

template <class geojson_conversion_policy, LoggingScenario scenario>
std::mutex GeojsonLogger<geojson_conversion_policy, scenario>::lock;

template <class geojson_conversion_policy, LoggingScenario scenario>
std::ofstream GeojsonLogger<geojson_conversion_policy, scenario>::ofs;

template <class geojson_conversion_policy, LoggingScenario scenario>
geojson_conversion_policy *GeojsonLogger<geojson_conversion_policy, scenario>::policy;
}
} // namespace osrm
