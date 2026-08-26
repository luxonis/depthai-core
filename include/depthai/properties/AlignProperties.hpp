#pragma once

#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Align
 */
struct AlignProperties : PropertiesSerializable<Properties, AlignProperties> {
    AlignConfig initialConfig;

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

    ~AlignProperties() override;
};

DEPTHAI_SERIALIZE_EXT(AlignProperties, initialConfig, numFramesPool, alignWidth, alignHeight);

}  // namespace dai
