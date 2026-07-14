#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

struct TestNodeProperties : PropertiesSerializable<Properties, TestNodeProperties> {
    bool reserved = false;

    ~TestNodeProperties() override;
};

DEPTHAI_SERIALIZE_EXT(TestNodeProperties, reserved);

}  // namespace dai
