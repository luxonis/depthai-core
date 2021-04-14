
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline() {
    dai::Pipeline p;

    auto camLeft = p.create<dai::node::MonoCamera>();
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);

    auto camRight = p.create<dai::node::MonoCamera>();
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    auto xoutLeft = p.create<dai::node::XLinkOut>();
    xoutLeft->setStreamName("left");
    auto xoutRight = p.create<dai::node::XLinkOut>();
    xoutRight->setStreamName("right");

    // Link plugins CAM -> XLINK
    camLeft->out.link(xoutLeft->input);
    camRight->out.link(xoutRight->input);

    return p;
}

int main() {
    using namespace std;

    dai::Pipeline p = createPipeline();
    dai::Device d(p);
    d.startPipeline();

    cv::Mat frame;
    auto leftQueue = d.getOutputQueue("left");
    auto rightQueue = d.getOutputQueue("right");

    while(1) {
        auto left = leftQueue->get<dai::ImgFrame>();
        cv::Mat LeftFrame = left->getFrame();
        cv::imshow("left", LeftFrame);
        auto right = rightQueue->get<dai::ImgFrame>();
        cv::Mat RightFrame = right->getFrame();
        cv::imshow("right", RightFrame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}
