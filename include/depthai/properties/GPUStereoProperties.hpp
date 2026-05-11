#pragma once

#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct GPUStereoProperties : PropertiesSerializable<Properties, GPUStereoProperties> {
    GPUStereoConfig initialConfig;
    ~GPUStereoProperties() override;
};

DEPTHAI_SERIALIZE_EXT(GPUStereoProperties, initialConfig);

}  // namespace dai
