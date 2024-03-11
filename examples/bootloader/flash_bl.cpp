#include <chrono>
#include <cstdio>
#include <string>

#include "XLink/XLink.h"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    using namespace std::chrono;
    auto cfg = dai::Device::Config{};
    cfg.board.usb.maxSpeed = dai::UsbSpeed::SUPER;

    auto eepromData = dai::EepromData{};
    eepromData.boardConf = "nIR-C00M00-00";
    eepromData.boardName = "BK2091";
    eepromData.boardRev = "REV1";
    eepromData.productName = "RX-CAMERA";
    eepromData.hardwareConf = "F0-FV00-BC000";
    eepromData.boardOptions = 3;
    eepromData.version = 7;
    //     {
    //     "batchName": "",
    //     "batchTime": 0,
    //     "boardConf": "nIR-C00M00-00",
    //     "boardName": "BK2091",
    //     "boardRev": "REV1",
    //     "productName": "RX-CAMERA",
    //     "boardCustom": "",
    //     "hardwareConf": "F0-FV00-BC000",
    //     "boardOptions": 3,
    //     "version": 7
    // }

    cfg.board.eepromData = eepromData;
    cfg.board.network.forceEnable = true;
    dai::Device device(cfg, cfg.board.usb.maxSpeed);
    std::function<void(float)> progressCb = [](float progress) { printf("Progress: %.2f\n", progress); };
    device.flashBootloader(dai::bootloader::Memory::FLASH, dai::bootloader::Type::NETWORK, progressCb);
}
