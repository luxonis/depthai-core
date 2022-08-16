#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutDepth->setStreamName("depth");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);


    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);
    stereo->depth.link(xoutDepth->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto qLeft = device.getOutputQueue("left", 4, false);
    auto qRight = device.getOutputQueue("right", 4, false);
    auto qDepth = device.getOutputQueue("depth", 4, false);

    while(true) {
        auto inLeft = qLeft->get<dai::ImgFrame>();
        auto frameLeft = inLeft->getCvFrame();
        cv::imshow("left", frameLeft);

        auto inRight = qRight->get<dai::ImgFrame>();
        auto frameRight = inRight->getCvFrame();
        cv::imshow("right", frameRight);

        auto inDepth = qDepth->get<dai::ImgFrame>();
        auto frameDepth = inDepth->getCvFrame();
        cv::imshow("depth", frameDepth);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
