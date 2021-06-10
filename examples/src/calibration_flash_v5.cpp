
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibBinaryFile(CALIB_PATH), boardConfigFile(BOARD_PATH);
    std::string calibBackUpPath("depthai_calib_backup.json");
    if(argc > 2) {
        calibBinaryFile = std::string(argv[1]);
        boardConfigFile = std::string(argv[2]);
    }

    dai::CalibrationHandler calibData(calibBinaryFile, boardConfigFile);
    dai::Device d;
    dai::CalibrationHandler deviceCalib = d.readCalibration();
    deviceCalib.eepromToJsonFile(calibBackUpPath);
    if(d.flashCalibration(calibData)) {
        std::cout << "Calibration flash Successful" << std::endl;
    } else {
        std::cout << "Calibration flash Failed!!!" << std::endl;
    }

    return 0;
}
