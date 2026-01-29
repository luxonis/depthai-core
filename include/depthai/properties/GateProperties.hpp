#pragma once

#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Gate.
 */

struct GateProperties : PropertiesSerializable<Properties, GateProperties> {
    GateControl initialConfig;

    ~GateProperties() override;
};

DEPTHAI_SERIALIZE_EXT(GateProperties, initialConfig);

}  // namespace dai
