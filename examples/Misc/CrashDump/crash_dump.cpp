#include <iostream>

#include "depthai/depthai.hpp"

// Main intention of this example is to test and verify the crash dump functionality works as intended
int main() try {
    std::cout << "This example crashes the device on pressing 'c' key. Press 'q' to quit." << std::endl;
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

    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
        if(key == 'c') {
            device.crashDevice();
        }
    }
    return 0;
} catch(const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
}
