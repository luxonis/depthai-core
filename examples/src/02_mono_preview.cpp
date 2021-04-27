#include <cstdio>
#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camLeft = pipeline.create<dai::node::MonoCamera>();
    auto camRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // Properties
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    // Linking
    camRight->out.link(xoutRight->input);
    camLeft->out.link(xoutLeft->input);

    dai::Device device(pipeline);
    device.startPipeline();

    // Output queues will be used to get the grayscale frames from the outputs defined above
    auto qLeft = device.getOutputQueue("left", 4, false);
    auto qRight = device.getOutputQueue("right", 4, false);

    while(true) {
        auto inLeft = qLeft->get<dai::ImgFrame>();
        auto inRight = qRight->get<dai::ImgFrame>();

        cv::imshow("left", inLeft->getCvFrame());
        cv::imshow("right", inRight->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}