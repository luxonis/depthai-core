#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageManip
 */
struct ImageManipPropertiesV2 : PropertiesSerializable<Properties, ImageManipPropertiesV2> {
    /**
     * Enable hardware accelerated image manipulation if set to HW. Only applied on RVC4.
     * This can cause some unexpected behavior when using multiple ImageManipV2 nodes in series.
     * Currently, the only operation affected is downscaling.
     */
    enum class Backend : uint8_t { CPU, HW };
    /**
     * Set performance mode for ImageManip with a tradeoff between performance and power consumption. Only applied on RVC4.
     * This only affects scaling NV12 and GRAY images.
     *  - PERFORMANCE: High performance, high power consumption. Uses the OpenCV backend.
     *  - BALANCED: Balanced performance and power consumption. Uses the FastCV backend configured for high performance where possible with a fallback to
     * OpenCV.
     *  - LOW_POWER: Low performance, low power consumption. Uses the FastCV backend configured for low power where possible with a fallback to OpenCV.
     */
    enum class PerformanceMode : uint8_t { PERFORMANCE, BALANCED, LOW_POWER };

    /// Initial configuration for ImageManip node
    ImageManipConfigV2 initialConfig;

    /// Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;

    /// Using HW backend can cause some unexpected behavior when using multiple ImageManipV2 nodes in series
    Backend backend = Backend::CPU;
    PerformanceMode performanceMode = PerformanceMode::PERFORMANCE;
};

DEPTHAI_SERIALIZE_EXT(ImageManipPropertiesV2, initialConfig, outputFrameSize, numFramesPool, backend, performanceMode);

}  // namespace dai
