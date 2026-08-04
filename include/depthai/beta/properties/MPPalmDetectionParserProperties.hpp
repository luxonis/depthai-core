#pragma once

#include <string>
#include <vector>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for MPPalmDetectionParser.
 */
struct MPPalmDetectionParserProperties : PropertiesSerializable<Properties, MPPalmDetectionParserProperties> {
    std::vector<std::string> outputLayerNames;
    float confidenceThreshold = 0.5f;
    float iouThreshold = 0.5f;
    int maxDetections = 100;
    int scale = 192;
    std::vector<std::string> labelNames{"Palm"};

    ~MPPalmDetectionParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(MPPalmDetectionParserProperties, outputLayerNames, confidenceThreshold, iouThreshold, maxDetections, scale, labelNames);

}  // namespace beta
}  // namespace dai
