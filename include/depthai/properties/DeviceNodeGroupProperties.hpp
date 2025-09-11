#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * @brief A dummy property struct for the DeviceNodeGroup node to comply with the DeviceNode API
 *
 */
struct DeviceNodeGroupProperties : PropertiesSerializable<Properties, DeviceNodeGroupProperties> {
    // Dummy property
    int dummy = 0;
};

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(DeviceNodeGroupProperties, dummy);

}  // namespace dai
