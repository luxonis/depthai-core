#pragma once

#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct GPUStereoProperties : PropertiesSerializable<Properties, GPUStereoProperties> {
    GPUStereoConfig initialConfig;
    int numFramesPool = 4;
    ~GPUStereoProperties() override;
};

DEPTHAI_SERIALIZE_EXT(GPUStereoProperties, initialConfig, numFramesPool);

}  // namespace dai
