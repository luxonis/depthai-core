#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setInterleaved(true);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Linking
    camRgb->preview.link(xoutRgb->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER);

    // Output queue will be used to get the rgb frames from the output defined above
    auto qRgb = device.getOutputQueue("rgb", 4, false);

    while(true) {
        auto inRgb = qRgb->get<dai::ImgFrame>();

        printf("w: %d, h: %d\n", inRgb->getWidth(), inRgb->getHeight());

        // Retrieve 'bgr' (opencv format) frame
        cv::imshow("rgb", inRgb->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    return 0;
}
