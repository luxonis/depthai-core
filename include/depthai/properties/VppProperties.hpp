#pragma once

#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/properties/Properties.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Specify properties for Vpp node
 */
struct VppProperties : PropertiesSerializable<Properties, VppProperties> {
    /**
     * Initial VPP configuration
     */
    VppConfig initialConfig;

    /**
     * Number of frames in pool for output frames
     */
    int numFramesPool = 4;

    virtual ~VppProperties();
};

DEPTHAI_SERIALIZE_EXT(VppProperties, initialConfig, numFramesPool);

}  // namespace dai
