
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    dai::Device d;

    dai::CalibrationHandler calibData = d.readCalibration();
    // calibData.eepromToJsonFile(filename);
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

    std::cout << "Intrinsics from getCameraIntrinsics function 1280 x 720  ->" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 1280, 720);

    for(auto row : intrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Intrinsics from getCameraIntrinsics function 720 x 450 ->" << std::endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 720);

    for(auto row : intrinsics) {
        for(auto val : row) std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Intrinsics from getCameraIntrinsics function 600 x 1280 ->" << std::endl;
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

    return 0;
}
