#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageManip
 */
struct ImageManipPropertiesV2 : PropertiesSerializable<Properties, ImageManipPropertiesV2> {
    /// Initial configuration for ImageManip node
    ImageManipConfigV2 initialConfig;

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
};

DEPTHAI_SERIALIZE_EXT(ImageManipPropertiesV2, initialConfig, outputFrameSize, numFramesPool, meshWidth, meshHeight, meshUri);

}  // namespace dai
