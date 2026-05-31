#pragma once

#include "depthai/pipeline/datatype/TofFusionConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for TofFusion node.
 */
struct TofFusionProperties : PropertiesSerializable<Properties, TofFusionProperties> {
    /**
     * Initial TofFusion config (compile-time defaults).
     */
    TofFusionConfig initialConfig;

    /**
     * Number of frames in output pool.
     */
    int numFramesPool = 4;

    ~TofFusionProperties() override;
};

DEPTHAI_SERIALIZE_EXT(TofFusionProperties, initialConfig, numFramesPool);

}  // namespace dai
