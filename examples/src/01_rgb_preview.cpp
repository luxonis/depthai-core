
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline() {
    dai::Pipeline p;

    auto camRgb = p.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(300, 300);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(true);

    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    // Link plugins CAM -> XLINK
    camRgb->preview.link(xlinkOut->input);

    return p;
}

int main() {
    using namespace std;

    dai::Pipeline p = createPipeline();
    dai::Device d(p);
    d.startPipeline();

    cv::Mat frame;
    auto preview = d.getOutputQueue("preview");

    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        if(imgFrame) {
            printf("Frame - w: %d, h: %d\n", imgFrame->getWidth(), imgFrame->getHeight());
            frame = cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3, imgFrame->getData().data());
            cv::imshow("preview", frame);
            int key = cv::waitKey(1);
            if(key == 'q') {
                return 0;
            }
        } else {
            std::cout << "Not ImgFrame" << std::endl;
        }
    }

    return 0;
}
