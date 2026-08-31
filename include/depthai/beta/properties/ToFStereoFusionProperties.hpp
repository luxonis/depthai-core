#pragma once

#include "depthai/beta/datatype/ToFStereoFusionConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai::beta {

struct ToFStereoFusionProperties : PropertiesSerializable<Properties, ToFStereoFusionProperties> {
    ToFStereoFusionConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(ToFStereoFusionProperties, initialConfig);

}  // namespace dai::beta
