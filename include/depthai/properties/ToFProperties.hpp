#pragma once

#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ToF
 */
struct ToFProperties : PropertiesSerializable<Properties, ToFProperties> {
    /**
     * Initial ToF config
     */
    ToFConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(ToFProperties, initialConfig);

}  // namespace dai
