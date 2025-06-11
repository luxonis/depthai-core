#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create and configure camera nodes
    auto left = pipeline.create<dai::node::Camera>();
    left->build(dai::CameraBoardSocket::CAM_B);

    auto right = pipeline.create<dai::node::Camera>();
    right->build(dai::CameraBoardSocket::CAM_C);

    // Create and configure sync node
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);  // Can also run on device

    // Link cameras to sync inputs
    left->requestFullResolutionOutput()->link(sync->inputs["left"]);
    right->requestFullResolutionOutput()->link(sync->inputs["right"]);

    // Create output queue
    auto outQueue = sync->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto messageGroup = outQueue->get<dai::MessageGroup>();
        if(messageGroup == nullptr) continue;

        auto leftMsg = messageGroup->get<dai::ImgFrame>("left");
        auto rightMsg = messageGroup->get<dai::ImgFrame>("right");

        std::cout << "Timestamps, message group " << messageGroup->getTimestamp().time_since_epoch().count() << std::endl;
        std::cout << "left " << leftMsg->getTimestamp().time_since_epoch().count() << std::endl;
        std::cout << "right " << rightMsg->getTimestamp().time_since_epoch().count() << std::endl;

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}