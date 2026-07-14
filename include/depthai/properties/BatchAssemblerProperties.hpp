#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

struct BatchAssemblerProperties : PropertiesSerializable<Properties, BatchAssemblerProperties> {
    bool reserved = false;

    ~BatchAssemblerProperties() override;
};

DEPTHAI_SERIALIZE_EXT(BatchAssemblerProperties, reserved);

}  // namespace dai
