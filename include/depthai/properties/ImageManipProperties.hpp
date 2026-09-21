#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageManip
 */
struct ImageManipProperties : PropertiesSerializable<Properties, ImageManipProperties> {
    /**
     * Backend preference for ImageManip on RVC4.
     *  - CPU: Run ImageManip on the CPU.
     *  - HW: Prefer the dedicated hardware image manipulation backend.
     *  - GPU: Prefer the GPU backend.
     *  - AUTO: Let the runtime select the backend automatically (GPU with CPU fallback).
     *
     * Hardware-accelerated backends can cause some unexpected behavior when using multiple
     * ImageManip nodes in series. Currently, the only operation affected is downscaling.
     */
    enum class Backend : uint8_t { CPU, HW, GPU, AUTO };
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
    ImageManipConfig initialConfig;

    /// Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;

    /// Maximum pool size for automatic calculation of pool size. Currently only used on RVC4
    unsigned int maxPoolSize = 400000000;

    /// Backend preference for ImageManip. Only applied on RVC4.
    Backend backend = Backend::AUTO;

    /// Performance mode for ImageManip. Only applied on RVC4.
    PerformanceMode performanceMode = PerformanceMode::PERFORMANCE;

    ~ImageManipProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ImageManipProperties, initialConfig, outputFrameSize, numFramesPool, maxPoolSize, backend, performanceMode);

}  // namespace dai
