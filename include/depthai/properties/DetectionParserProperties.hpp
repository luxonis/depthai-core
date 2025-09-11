#pragma once

#include <vector>

#include "depthai/common/DetectionParserOptions.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

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

#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(DetectionParserProperties, numFramesPool, networkInputs, parser);

}  // namespace dai
