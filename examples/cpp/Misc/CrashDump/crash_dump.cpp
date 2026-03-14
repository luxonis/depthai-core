#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Main intention of this example is to test and verify the crash dump functionality works as intended
int main() try {
    std::cout << "This example crashes the device on pressing 'c' key. Press 'q' to quit." << std::endl;
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    auto video = camRgb->requestOutput({1920, 1080})->createOutputQueue(1, false);

    auto device = pipeline.getDefaultDevice();

    bool running = true;
    while(running) {
        auto videoIn = video->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            running = false;
        }
        if(key == 'c') {
            device->crashDevice();
            running = false;
        }
    }
    return 0;
} catch(const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
}
