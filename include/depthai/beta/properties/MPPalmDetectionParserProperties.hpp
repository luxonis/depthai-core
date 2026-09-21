#pragma once

#include <string>
#include <vector>

#include "depthai/beta/datatype/MPPalmDetectionParserConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for MPPalmDetectionParser.
 */
struct MPPalmDetectionParserProperties : PropertiesSerializable<Properties, MPPalmDetectionParserProperties> {
    std::vector<std::string> outputLayerNames;
    MPPalmDetectionParserConfig initialConfig;
    int scale = 192;
    std::vector<std::string> labelNames{"Palm"};

    ~MPPalmDetectionParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(MPPalmDetectionParserProperties, outputLayerNames, initialConfig, scale, labelNames);

}  // namespace beta
}  // namespace dai
