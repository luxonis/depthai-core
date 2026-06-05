#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for SplitterNode.
 *
 * This is a placeholder until the splitting behavior is implemented.
 */
struct SplitterNodeProperties : PropertiesSerializable<Properties, SplitterNodeProperties> {
    int reserved = 0;

    ~SplitterNodeProperties() override;
};

DEPTHAI_SERIALIZE_EXT(SplitterNodeProperties, reserved);

}  // namespace dai
