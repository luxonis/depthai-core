#pragma once

#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct GPUStereoProperties : PropertiesSerializable<Properties, GPUStereoProperties> {
    GPUStereoConfig initialConfig;
    /// Rectify the inputs inside the node (on the EVA warp engine) using the calibration they carry.
    bool rectification = true;
    ~GPUStereoProperties() override;
};

DEPTHAI_SERIALIZE_EXT(GPUStereoProperties, initialConfig, rectification);

}  // namespace dai
