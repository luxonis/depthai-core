#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for MLSDParser.
 */
struct MLSDParserProperties : PropertiesSerializable<Properties, MLSDParserProperties> {
    std::string outputLayerTPMap;
    std::string outputLayerHeat;
    int topK = 200;
    float scoreThreshold = 0.10f;
    float distanceThreshold = 20.0f;
    std::pair<std::uint32_t, std::uint32_t> inputSize{512, 512};

    ~MLSDParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(MLSDParserProperties, outputLayerTPMap, outputLayerHeat, topK, scoreThreshold, distanceThreshold, inputSize);

}  // namespace beta
}  // namespace dai
