#pragma once

#include "depthai/common/EepromData.hpp"
#include "depthai/common/Interpolation.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Warp
 */
struct WarpProperties : PropertiesSerializable<Properties, WarpProperties> {
    /// Output width
    int outputWidth = 0;

    /// Output height
    int outputHeight = 0;

    /// Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;

    /// Custom warp mesh width. Set to zero to disable
    int meshWidth = 0;
    /// Custom warp mesh height. Set to zero to disable.
    int meshHeight = 0;
    /// Custom warp mesh uri. Set to empty string to disable.
    std::string meshUri = "";

    /// Warp HW IDs to use, if empty, use auto/default
    std::vector<int> warpHwIds;

    Interpolation interpolation = Interpolation::AUTO;

#if defined(__clang__)
    ~WarpProperties() override;
#else
    virtual ~WarpProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(WarpProperties, outputWidth, outputHeight, outputFrameSize, numFramesPool, meshWidth, meshHeight, meshUri, warpHwIds, interpolation);

}  // namespace dai
