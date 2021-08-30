#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    using namespace std;

    // Connect Device
    dai::Device device;

    dai::CalibrationHandler calibData = device.readCalibration();
    // calibData.eepromToJsonFile(filename);
    std::vector<std::vector<float>> intrinsics;
    int width, height;

    cout << "Intrinsics from defaultIntrinsics function" << endl;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Width -> " << width << endl;
    cout << "Height -> " << height << endl;

    cout << "Intrinsics from getCameraIntrinsics function full resolution ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from getCameraIntrinsics function 1280 x 720  ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 1280, 720);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from getCameraIntrinsics function 720 x 450 ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 720);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from getCameraIntrinsics function 600 x 1280 ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 600, 1280);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    std::vector<std::vector<float>> extrinsics;

    cout << "Extrinsics from left->right test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from right->left test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from right->rgb test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::RGB);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from rgb->right test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RGB, dai::CameraBoardSocket::RIGHT);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from left->rgb test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RGB);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    return 0;
}
