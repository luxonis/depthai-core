#pragma once

#include <vector>

#include "depthai/common/Interpolation.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageAlign
 */
struct ImageAlignProperties : PropertiesSerializable<Properties, ImageAlignProperties> {
    ImageAlignConfig initialConfig;

    /// Num frames in output pool
    int numFramesPool = 4;

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

    ~ImageAlignProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ImageAlignProperties, initialConfig, numFramesPool, alignWidth, alignHeight, warpHwIds, interpolation, outKeepAspectRatio, numShaves);

}  // namespace dai
