#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/**
 * Specify properties for EdgeDetector
 */
struct EdgeDetectorProperties : PropertiesSerializable<Properties, EdgeDetectorProperties> {
    /// Initial edge detector config
    EdgeDetectorConfig initialConfig;

    /**
     * Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
     */
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;
};

DEPTHAI_SERIALIZE_EXT(EdgeDetectorProperties, initialConfig, outputFrameSize, numFramesPool);

#ifdef __clang__
#pragma clang diagnostic pop
#endif

}  // namespace dai
