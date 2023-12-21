#pragma once

#include <vector>

#include "depthai/common/DetectionParserOptions.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for DetectionParser
 */
struct DetectionParserProperties : PropertiesSerializable<Properties, DetectionParserProperties> {
    /// Num frames in output pool
    int numFramesPool = 8;

    /// Network inputs
    std::unordered_map<std::string, TensorInfo> networkInputs;

    /// Options for parser
    DetectionParserOptions parser;
};

DEPTHAI_SERIALIZE_EXT(DetectionParserProperties, numFramesPool, networkInputs, parser);

}  // namespace dai
