#pragma once

#include <string>

#include "depthai/beta/datatype/PPTextDetectionParserConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for PPTextDetectionParser.
 */
struct PPTextDetectionParserProperties : PropertiesSerializable<Properties, PPTextDetectionParserProperties> {
    std::string outputLayerName;
    PPTextDetectionParserConfig initialConfig;

    ~PPTextDetectionParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(PPTextDetectionParserProperties, outputLayerName, initialConfig);

}  // namespace beta
}  // namespace dai
