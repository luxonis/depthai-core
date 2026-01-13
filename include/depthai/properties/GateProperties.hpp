#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Gate.
 */

struct GateProperties : PropertiesSerializable<Properties, GateProperties> {
    int sleepingTimeMs = 50;  // sleep 50 ms between each inputControl reading

    ~GateProperties() override;
};

DEPTHAI_SERIALIZE_EXT(GateProperties, sleepingTimeMs);

}  // namespace dai
