#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    dai::Device device;

    std::cout << "Is EEPROM available: " << device.isEepromAvailable() << std::endl;

    try {
        nlohmann::json j = device.readCalibration2().getEepromData();
        std::cout << "User calibration: " << j.dump(4) << std::endl << std::endl;
    } catch(const std::exception& ex) {
        std::cout << "No user calibration: " << ex.what() << std::endl;
    }

    try {
        nlohmann::json j = device.readFactoryCalibration().getEepromData();
        std::cout << "Factory calibration: " << j.dump(4) << std::endl << std::endl;
    } catch(const std::exception& ex) {
        std::cout << "No factory calibration: " << ex.what() << std::endl;
    }

    return 0;
}
