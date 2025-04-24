#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageManip
 */
struct ImageManipPropertiesV2 : PropertiesSerializable<Properties, ImageManipPropertiesV2> {
    enum class Backend : uint8_t { CPU, HW };
    enum class PerformanceMode : uint8_t { AUTO, PERFORMANCE, LOW_POWER };

    /// Initial configuration for ImageManip node
    ImageManipConfigV2 initialConfig;

    /// Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;

    /// Using HW backend can cause some unexpected behavior when using multiple ImageManipV2 nodes in series
    Backend backend = Backend::CPU;
    PerformanceMode performanceMode = PerformanceMode::AUTO;
};

DEPTHAI_SERIALIZE_EXT(ImageManipPropertiesV2, initialConfig, outputFrameSize, numFramesPool, backend, performanceMode);

}  // namespace dai
