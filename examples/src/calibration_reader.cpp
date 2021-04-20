
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

dai::Pipeline createCameraPipeline() {
    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->input);

    return p;
}

int main(int argc, char** argv) {
    std::string filename;

    std::cout << "Number of args" << argc << std::endl;
    if(argc == 2) {
        filename = std::string(argv[1]);
    } else {
        throw std::runtime_error("Required destination json path file");
    }

    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);

    dai::CalibrationHandler calibData = d.getCalibration();
    calibData.eepromToJsonFile(filename);
    std::vector<std::vector<float>> intrinsics;
    int width, height;

    std::cout << "Intrinsics from defaultIntrinsics function" << std::endl;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Width -> " << width << std::endl;
    std::cout << "Height -> " << height << std::endl;

    std::cout << "Intrinsics from getCameraIntrinsics function full resolution ->" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Intrinsics from getCameraIntrinsics function 720 x 1280 ->" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 720);

    for(auto row : intrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Intrinsics from getCameraIntrinsics function 600 x 1280 2 test ->" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 600, 1280);

    for(auto row : intrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::vector<std::vector<float>> extrinsics;

    std::cout << "Extrinsics from left->right test ->" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT);

    for(auto row : extrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Extrinsics from right->left test ->" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);

    for(auto row : extrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Extrinsics from right->rgb test ->" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::RGB);

    for(auto row : extrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Extrinsics from rgb->right test ->" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RGB, dai::CameraBoardSocket::RIGHT);

    for(auto row : extrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Extrinsics from left->rgb test ->" << std::endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RGB);

    for(auto row : extrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    d.startPipeline();
    auto preview = d.getOutputQueue("preview");
    cv::Mat frame;

    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        if(imgFrame) {
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
