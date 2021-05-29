
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibPth(CALIB_PATH);
    if(argc > 1) {
        calibPth = std::string(argv[1]);
    }

    dai::CalibrationHandler calibData(calibPth);
    dai::Device d;
    if(d.flashCalibration(calibData)) {
        std::cout << "Calibration flash Successful" << std::endl;
    } else {
        std::cout << "Calibration flash Failed!!!" << std::endl;
    }

    return 0;
}
