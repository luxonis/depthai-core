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

#if defined(__clang__)
    ~DeviceNodeGroupProperties() override;
#else
    virtual ~DeviceNodeGroupProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(DeviceNodeGroupProperties, dummy);

}  // namespace dai
