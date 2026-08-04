#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for SCRFDParser.
 */
struct SCRFDParserProperties : PropertiesSerializable<Properties, SCRFDParserProperties> {
    std::vector<std::string> outputLayerNames;
    float confidenceThreshold = 0.5f;
    float iouThreshold = 0.5f;
    int maxDetections = 100;
    std::pair<std::uint32_t, std::uint32_t> inputSize{640, 640};
    std::vector<std::int64_t> featStrideFpn{8, 16, 32};
    std::int64_t numAnchors = 2;
    std::vector<std::string> labelNames{"Face"};

    ~SCRFDParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(
    SCRFDParserProperties, outputLayerNames, confidenceThreshold, iouThreshold, maxDetections, inputSize, featStrideFpn, numAnchors, labelNames);

}  // namespace beta
}  // namespace dai
