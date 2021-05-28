
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibBinaryFile(CALIB_PATH), boardConfigFile(BOARD_PATH);
    if(argc > 2) {
        calibBinaryFile = std::string(argv[1]);
        boardConfigFile = std::string(argv[2]);
    }

    dai::Device d;
    dai::CalibrationHandler calibData(calibBinaryFile, boardConfigFile);
    if(d.flashCalibration(calibData)) {
        std::cout << "Calibration flash Successfull" << std::endl;
    } else {
        std::cout << "Calibration flash Failed!!!" << std::endl;
    }

    return 0;
}
