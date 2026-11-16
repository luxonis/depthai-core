#pragma once

#include "depthai/beta/datatype/ToFStereoFusionConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai::beta {

struct ToFStereoFusionProperties : PropertiesSerializable<Properties, ToFStereoFusionProperties> {
    ToFStereoFusionConfig initialConfig;

    ~ToFStereoFusionProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(ToFStereoFusionProperties, initialConfig);

}  // namespace dai::beta

extern template struct dai::PropertiesSerializable<dai::Properties, dai::beta::ToFStereoFusionProperties>;
