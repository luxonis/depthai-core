
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline() {
    dai::Pipeline p;

    auto camLeft = p.create<dai::node::MonoCamera>();
    auto camRight = p.create<dai::node::MonoCamera>();
    auto xout = p.create<dai::node::XLinkOut>();
    auto depth = p.create<dai::node::StereoDepth>();

    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    depth->setConfidenceThreshold(200);
    //depth->setMedianFilter(median);

    // Link plugins CAM -> XLINK
    camLeft->out.link(depth->left);
    camRight->out.link(depth->right);

    xout->setStreamName("disparity");
    depth->disparity.link(xout->input);

    return p;
}

int main() {
    using namespace std;

    dai::Pipeline p = createPipeline();
    dai::Device d(p);
    d.startPipeline();

    auto dispQueue = d.getOutputQueue("disparity", 4, false);

    while(1) {
        auto disparity = dispQueue->get<dai::ImgFrame>();
        cv::Mat disp(disparity->getHeight(), disparity->getWidth(), CV_8UC1, disparity->getData().data());
        disp.convertTo(disp, CV_8UC1, 3.0);
        cv::imshow("disparity", disp);
        cv::Mat disp_color;
        cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);
        cv::imshow("disparity_color", disp_color);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}
