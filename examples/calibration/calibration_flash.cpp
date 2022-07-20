#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibJsonFile(CALIB_PATH);
    std::string calibBackUpFile("depthai_calib_backup.json");
    if(argc > 1) {
        calibJsonFile = std::string(argv[1]);
    }

    // Connect device
    dai::Device device;

    dai::CalibrationHandler deviceCalib = device.readCalibration();
    deviceCalib.eepromToJsonFile(calibBackUpFile);
    std::cout << "Calibration Data on the device is backed up at:" << calibBackUpFile << std::endl;
    dai::CalibrationHandler calibData(calibJsonFile);

    if(device.flashCalibration(calibData)) {
        std::cout << "Calibration Flash Successful" << std::endl;
    } else {
        std::cout << "Calibration Flash Failed!!!" << std::endl;
    }

    return 0;
}
