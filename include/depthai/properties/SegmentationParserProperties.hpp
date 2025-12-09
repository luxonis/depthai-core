#pragma once

#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for SegmentationParser
 */
struct SegmentationParserProperties : PropertiesSerializable<Properties, SegmentationParserProperties> {
    SegmentationParserConfig parserConfig;

    std::string networkOutputName = "";
    bool classesInOneLayer = false;

    ~SegmentationParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(SegmentationParserProperties, parserConfig, networkOutputName, classesInOneLayer);

}  // namespace dai
