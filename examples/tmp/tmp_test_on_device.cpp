#include <iostream>
#include "depthai/device/DeviceBaseNew.hpp"
#include "depthai/common/CameraBoardSocket.hpp"

int main() {
    // Create an instance of DeviceBaseNew
    dai::DeviceBaseNew device;

    // Get the connected cameras
    std::vector<dai::CameraBoardSocket> cameras = device.getConnectedCameras();

    // Print the connected cameras
    std::cout << "Connected cameras: ";
    for(const auto& camera : cameras) {
        std::cout << static_cast<int>(camera) << " ";  // Assuming CameraBoardSocket is an enum
    }
    std::cout << std::endl;

    return 0;
}