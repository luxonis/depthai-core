#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for RFDETRParser.
 */
struct RFDETRParserProperties : PropertiesSerializable<Properties, RFDETRParserProperties> {
    float confidenceThreshold = 0.5f;
    int maxDetections = 300;
    std::vector<std::string> labelNames;
    float maskConfidence = 0.5f;
    std::vector<std::string> outputLayerNames;
    std::optional<std::pair<std::uint32_t, std::uint32_t>> inputSize;

    ~RFDETRParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RFDETRParserProperties, confidenceThreshold, maxDetections, labelNames, maskConfidence, outputLayerNames, inputSize);

}  // namespace beta
}  // namespace dai
