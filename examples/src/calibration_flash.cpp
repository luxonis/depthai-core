
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calib_pth(CALIB_PATH);
    if(argc > 1) {
        calib_pth = std::string(argv[1]);
    }

    dai::CalibrationHandler calibData(calib_pth);
    dai::Device d;
    if(d.flashCalibration(calibData)) {
        std::cout << "Calibration flash Successfull" << std::endl;
    } else {
        std::cout << "Calibration flash Failed!!!" << std::endl;
    }

    return 0;
}
