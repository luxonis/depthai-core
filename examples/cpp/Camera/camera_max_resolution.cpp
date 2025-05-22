#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto cam = pipeline.create<dai::node::Camera>()->build();
    // In some cases (IMX586), this requires an 8k screen to be able to see the full resolution at once
    auto videoQueue = cam->requestFullResolutionOutput()->createOutputQueue();

    // Start pipeline
    pipeline.start();

    while(true) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) continue;

        cv::imshow("video", videoIn->getCvFrame());

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}