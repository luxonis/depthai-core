#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    // Query all available devices (USB and POE OAK cameras)
    auto deviceInfoVec = dai::DeviceBootloader::getAllAvailableDevices();

    if(deviceInfoVec.size() == 0) {
        std::cout << "DepthAI couldn't find any available OAK device!\n";
        return 0;
    }

    for(auto& deviceInfo : deviceInfoVec) {
        std::string state{""};
        switch(deviceInfo.state) {
            case X_LINK_ANY_STATE:
                state = "ANY_STATE";
                break;
            case X_LINK_BOOTED:
                state = "BOOTED";
                break;
            case X_LINK_UNBOOTED:
                state = "UNBOOTED";
                break;
            case X_LINK_BOOTLOADER:
                state = "BOOTLOADER";
                break;
            case X_LINK_FLASH_BOOTED:
                state = "FLASH_BOOTED";
                break;
            default:
                state = "UNKNOWN STATE";
        }
        std::cout << "Found device " << deviceInfo.name << ", MxId: " << deviceInfo.mxid << ", State: " << state << std::endl;
    }

    // Connect to a specific device. We will just take the first one
    std::cout << "Booting the first available camera (" << deviceInfoVec.at(0).name << ")\n";

    const auto usbSpeed = dai::UsbSpeed::SUPER;
    dai::Device device(dai::Pipeline(), deviceInfoVec.at(0), usbSpeed);
    std::cout << "Available camera sensors: \n";
    for(auto& el : device.getCameraSensorNames()) {
        std::cout << el.second << std::endl;
    }
    auto calib = device.readCalibration();
    auto eeprom = calib.getEepromData();
    if(eeprom.boardName != "") {
        std::cout << "   >>> Board name:" << eeprom.boardName << std::endl;
    }
    if(eeprom.productName != "") {
        std::cout << "   >>> Product name:" << eeprom.productName << std::endl;
    }

    return 0;
}
