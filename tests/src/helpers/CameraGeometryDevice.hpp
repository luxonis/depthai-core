#pragma once

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "depthai/device/Device.hpp"

namespace dai {
namespace test {
inline std::shared_ptr<Device> openGeometryDevice() {
    const auto* selectedId = std::getenv("DEPTHAI_DEVICE_ID_LIST");
    std::shared_ptr<Device> device;
    if(selectedId != nullptr && selectedId[0] != '\0' && std::string(selectedId).find(',') == std::string::npos) {
        // Explicit selection avoids auto-discovery falling back to a different
        // booted device when the selected camera disappears between sessions.
        device = std::make_shared<Device>(DeviceInfo(selectedId));
        if(device->getDeviceId() != selectedId) throw std::runtime_error("Geometry test connected to a different device");
    } else {
        const auto [found, candidate] = Device::getAnyAvailableDevice();
        if(!found || (candidate.state == X_LINK_BOOTED && candidate.protocol != X_LINK_LOCAL_SHDMEM)) {
            throw std::runtime_error("No idle device available for the geometry test");
        }
        device = std::make_shared<Device>(candidate);
    }
    if(std::getenv("DEPTHAI_GEOMETRY_EXTENDED") != nullptr) std::cout << "GEOMETRY_DEVICE id=" << device->getDeviceId() << std::endl;
    return device;
}
}  // namespace test
}  // namespace dai
