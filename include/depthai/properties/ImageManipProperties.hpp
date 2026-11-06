#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/properties/FramePoolProperties.hpp"
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

    /// Deprecated frame capacity fallback for pool.outputFrameSize, in bytes; zero means automatic.
    [[deprecated("Use pool.outputFrameSize instead")]] int outputFrameSize = 0;

    /// Deprecated frame count fallback for pool.numFramesPool; zero means automatic.
    [[deprecated("Use pool.numFramesPool instead")]] int numFramesPool = 0;

    /// Deprecated memory limit fallback for pool.maxPoolSize, in bytes; zero means automatic.
    [[deprecated("Use pool.maxPoolSize instead")]] unsigned int maxPoolSize = 0;

    /// Backend preference for ImageManip. Only applied on RVC4.
    Backend backend = Backend::AUTO;

    /// Performance mode for ImageManip. Only applied on RVC4.
    PerformanceMode performanceMode = PerformanceMode::PERFORMANCE;

    /// Pool configuration. Nonzero fields override legacy fields; zero falls back to the legacy value.
    /// Legacy fields also default to zero (automatic). Prefer pool for new code.
    FramePoolProperties pool{};

    ~ImageManipProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ImageManipProperties, initialConfig, outputFrameSize, numFramesPool, maxPoolSize, backend, performanceMode, pool);

}  // namespace dai
