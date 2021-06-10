
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibPth(CALIB_PATH);
    std::string calibBackUpPath("depthai_calib_backup.json");
    if(argc > 1) {
        calibPth = std::string(argv[1]);
    }

    dai::CalibrationHandler calibData(calibPth);
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
