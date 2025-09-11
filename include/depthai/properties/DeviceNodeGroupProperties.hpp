#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * @brief A dummy property struct for the DeviceNodeGroup node to comply with the DeviceNode API
 *
 */
struct DeviceNodeGroupProperties : PropertiesSerializable<Properties, DeviceNodeGroupProperties> {
    // Dummy property
    int dummy = 0;

    ~DeviceNodeGroupProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DeviceNodeGroupProperties, dummy);

}  // namespace dai
