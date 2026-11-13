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

DEPTHAI_SERIALIZE_EXT_DECL(DeviceNodeGroupProperties, dummy);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::DeviceNodeGroupProperties>;
