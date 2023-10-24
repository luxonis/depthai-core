#pragma once
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
namespace dai {
class DeviceBaseImpl {
   public:
    DeviceBaseImpl() = default;
    virtual ~DeviceBaseImpl();
    virtual std::vector<CameraBoardSocket> getConnectedCameras();
};

class DeviceBaseImplMock : public DeviceBaseImpl {
   public:
    DeviceBaseImplMock() = default;
    virtual ~DeviceBaseImplMock();
    std::vector<CameraBoardSocket> getConnectedCameras() override;
};
}  // namespace dai