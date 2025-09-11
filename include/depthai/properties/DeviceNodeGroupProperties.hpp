#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/**
 * @brief A dummy property struct for the DeviceNodeGroup node to comply with the DeviceNode API
 *
 */
struct DeviceNodeGroupProperties : PropertiesSerializable<Properties, DeviceNodeGroupProperties> {
    // Dummy property
    int dummy = 0;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(DeviceNodeGroupProperties, dummy);

}  // namespace dai
