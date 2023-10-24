#pragma once
#include <memory>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {
class DeviceBaseImpl; // Forward declare the implementation class
class DeviceBaseNew {
   public:
    // Delete copy constructor and assignment operator
    DeviceBaseNew(const DeviceBaseNew&) = delete;
    DeviceBaseNew& operator=(const DeviceBaseNew&) = delete;
    DeviceBaseNew();
    virtual ~DeviceBaseNew();

    /**
     * Get cameras that are connected to the device
     *
     * @returns Vector of connected cameras
     */
    std::vector<CameraBoardSocket> getConnectedCameras();

   private:
    std::unique_ptr<DeviceBaseImpl> pimpl;
    void *shared_library_handle;
};

}  // namespace dai