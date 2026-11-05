#pragma once

#include <vector>

#include "depthai/common/Interpolation.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/properties/FramePoolProperties.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageAlign
 */
struct ImageAlignProperties : PropertiesSerializable<Properties, ImageAlignProperties> {
    ImageAlignConfig initialConfig;

    /// Deprecated frame count fallback for pool.numFramesPool; zero means automatic.
    [[deprecated("Use pool.numFramesPool instead")]] int numFramesPool = 0;

    /**
     * Optional output width
     */
    int alignWidth = 0;
    /**
     * Optional output height
     */
    int alignHeight = 0;

    /// Warp HW IDs to use, if empty, use auto/default
    std::vector<int> warpHwIds;
    using Interpolation = dai::Interpolation;
    /// Interpolation type to use
    Interpolation interpolation = Interpolation::AUTO;
    /**
     * Whether to keep aspect ratio of the input or not
     */
    bool outKeepAspectRatio = true;

    /**
     * Number of shaves reserved.
     */
    std::int32_t numShaves = 2;

    /// Pool configuration. Nonzero fields override legacy fields; zero falls back to the legacy value.
    /// Legacy fields also default to zero (automatic). Prefer pool for new code.
    FramePoolProperties pool{};

    ~ImageAlignProperties() override;
};

DEPTHAI_SERIALIZE_EXT(
    ImageAlignProperties, initialConfig, numFramesPool, alignWidth, alignHeight, warpHwIds, interpolation, outKeepAspectRatio, numShaves, pool);

}  // namespace dai
