#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for GatherData.
 *
 * This is a placeholder until the collection behavior is implemented.
 */
struct GatherDataProperties : PropertiesSerializable<Properties, GatherDataProperties> {
    int reserved = 0;

    ~GatherDataProperties() override;
};

DEPTHAI_SERIALIZE_EXT(GatherDataProperties, reserved);

}  // namespace dai
