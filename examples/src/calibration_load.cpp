
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

dai::Pipeline createCameraPipeline() {
    dai::Pipeline p;

    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    // monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    // monoRight->setFps(5.0);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    return p;
}

int main(int argc, char** argv) {
    std::string calibPth(CALIB_PATH);
    if(argc > 1) {
        calibPth = std::string(argv[1]);
    }

    dai::CalibrationHandler calibData(calibPth);
    dai::Pipeline p = createCameraPipeline();
    p.setCalibrationData(calibData);
    dai::Device d(p);

    auto depthQueue = d.getOutputQueue("depth");
    cv::Mat frame;

    while(1) {
        auto imgFrame = depthQueue->get<dai::ImgFrame>();
        if(imgFrame) {
            frame = cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_16UC1, imgFrame->getData().data());
            cv::imshow("depth", frame);
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
