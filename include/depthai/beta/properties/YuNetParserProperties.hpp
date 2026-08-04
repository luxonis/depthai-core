#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for YuNetParser.
 */
struct YuNetParserProperties : PropertiesSerializable<Properties, YuNetParserProperties> {
    std::string locOutputLayerName;
    std::string confOutputLayerName;
    std::string iouOutputLayerName;
    float confidenceThreshold = 0.8f;
    float iouThreshold = 0.3f;
    int maxDetections = 5000;
    std::optional<std::pair<std::uint32_t, std::uint32_t>> inputSize;
    std::vector<std::string> labelNames{"Face"};

    ~YuNetParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(YuNetParserProperties,
                      locOutputLayerName,
                      confOutputLayerName,
                      iouOutputLayerName,
                      confidenceThreshold,
                      iouThreshold,
                      maxDetections,
                      inputSize,
                      labelNames);

}  // namespace beta
}  // namespace dai
