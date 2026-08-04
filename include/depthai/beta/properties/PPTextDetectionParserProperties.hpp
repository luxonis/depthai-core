#pragma once

#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for PPTextDetectionParser.
 */
struct PPTextDetectionParserProperties : PropertiesSerializable<Properties, PPTextDetectionParserProperties> {
    std::string outputLayerName;
    float confidenceThreshold = 0.5f;
    float maskThreshold = 0.25f;
    int maxDetections = 100;

    ~PPTextDetectionParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(PPTextDetectionParserProperties, outputLayerName, confidenceThreshold, maskThreshold, maxDetections);

}  // namespace beta
}  // namespace dai
