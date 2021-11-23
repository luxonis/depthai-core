#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(1000, 500);
    camRgb->setInterleaved(false);
    auto maxFrameSize = camRgb->getPreviewHeight() * camRgb->getPreviewHeight() * 3;

    // Warp preview frame
    auto manip1 = pipeline.create<dai::node::ImageManip>();
    // Create a custom warp mesh
    dai::Point2f tl(20, 20);
    dai::Point2f tr(460, 20);
    dai::Point2f ml(100, 250);
    dai::Point2f mr(400, 250);
    dai::Point2f bl(20, 460);
    dai::Point2f br(460, 460);
    manip1->setWarpMesh({tl,tr,ml,mr,bl,br}, 2, 3);

    manip1->setMaxOutputFrameSize(maxFrameSize);
    camRgb->preview.link(manip1->inputImage);

    auto xout1 = pipeline.create<dai::node::XLinkOut>();
    xout1->setStreamName("out1");
    manip1->out.link(xout1->input);

    dai::Device device(pipeline);
    auto q1 = device.getOutputQueue("out1", 8, false);
    while(true) {
        auto in1 = q1->get<dai::ImgFrame>();
        if(in1) {
            cv::imshow("Warped preview", in1->getCvFrame());
        }
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}


