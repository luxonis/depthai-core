#include <cassert>
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
    auto videoOutput = cam->requestOutput(std::make_pair(640, 400));
    auto videoQueue = videoOutput->createOutputQueue();

    pipeline.build();

    // Optionally update internal settings
    // Note: xlink bridges are only generated after pipeline.build() is called
    auto xlinkBridge = videoOutput->getXLinkBridge();
    assert(xlinkBridge != nullptr);
    assert(xlinkBridge->xLinkOut != nullptr);
    xlinkBridge->xLinkOut->setFpsLimit(3.0);

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