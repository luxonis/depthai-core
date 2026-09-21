#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/common/KeypointsListT.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for KeypointParser.
 */
struct KeypointParserProperties : PropertiesSerializable<Properties, KeypointParserProperties> {
    std::string outputLayerName;
    float scaleFactor = 1.0f;
    std::optional<std::int64_t> nKeypoints;
    std::optional<float> scoreThreshold;
    std::vector<std::string> labelNames;
    std::vector<Edge> edges;

    ~KeypointParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(KeypointParserProperties, outputLayerName, scaleFactor, nKeypoints, scoreThreshold, labelNames, edges);

}  // namespace beta
}  // namespace dai
