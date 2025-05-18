#include <iostream>
#include "depthai/depthai.hpp"
#include <opencv2/opencv.hpp>

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto cam = pipeline.create<dai::node::Camera>()->build();
    // In some cases (IMX586), this requires an 8k screen to be able to see the full resolution at once
    auto videoQueue = cam->requestFullResolutionOutput()->createOutputQueue();

    pipeline.start();
    while(true) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn != nullptr) {
            cv::imshow("video", videoIn->getCvFrame());
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
} 