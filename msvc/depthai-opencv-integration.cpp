#include <iostream>
#include "depthai/depthai.hpp"

int main() try {

    // Create pipeline
    dai::Pipeline pipeline;
    
    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // Properties
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    // Linking
    monoRight->out.link(xoutRight->input);
    monoLeft->out.link(xoutLeft->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the grayscale frames from the outputs defined above
    auto qLeft = device.getOutputQueue("left", 4, false);
    auto qRight = device.getOutputQueue("right", 4, false);

    while (true) {
        // Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        auto inLeft = qLeft->tryGet<dai::ImgFrame>();
        auto inRight = qRight->tryGet<dai::ImgFrame>();
        cv::Mat leftMat, rightMat;

        if (inLeft && !(leftMat = inLeft->getCvFrame()).empty()) {
            cv::imshow("left", leftMat);
        }

        if(inRight && !(rightMat = inRight->getCvFrame()).empty()) {
            cv::imshow("right", rightMat);
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            return 0;
        }
    }

    return 0;

} catch(const std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
}
