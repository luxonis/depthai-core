#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camMono = pipeline.create<dai::node::MonoCamera>();
    auto camMono2 = pipeline.create<dai::node::MonoCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutRgb2 = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("mono");
    xoutRgb2->setStreamName("mono2");

    camMono->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camMono2->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Linking
    camMono->out.link(xoutRgb->input);
    camMono2->out.link(xoutRgb2->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER);

    // Output queue will be used to get the rgb frames from the output defined above
    auto qRgb = device.getOutputQueue("mono", 4, false);
    auto qRgb2 = device.getOutputQueue("mono2", 4, false);

    while(true) {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto inRgb2 = qRgb2->get<dai::ImgFrame>();

        // Retrieve 'bgr' (opencv format) frame
        cv::imshow("mono", inRgb->getCvFrame());
        cv::imshow("mono2", inRgb2->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    return 0;
}
