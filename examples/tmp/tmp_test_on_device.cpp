#include <iostream>
#include "depthai/device/DeviceBaseNew.hpp"
#include "depthai/common/CameraBoardSocket.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// int main() {
//     // Create an instance of DeviceBaseNew
//     dai::DeviceBaseNew device;

//     // Get the connected cameras
//     std::vector<dai::CameraBoardSocket> cameras = device.getConnectedCameras();

//     // Print the connected cameras
//     std::cout << "Connected cameras: ";
//     for(const auto& camera : cameras) {
//         std::cout << static_cast<int>(camera) << " ";  // Assuming CameraBoardSocket is an enum
//     }
//     std::cout << std::endl;

//     return 0;
// }

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1920, 1080);

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto video = device.getOutputQueue("video");
    int count = 0;
    auto startTime = std::chrono::steady_clock::now();
    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();
        count++;
        // Check how much time has elapsed
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        if(elapsed > 1000) {
            std::cout << "FPS: " << count << std::endl;
            count = 0;
            startTime = currentTime;
        }
    }
    return 0;
}