#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>

#include "depthai/depthai.hpp"

int main() {
    bool found = false;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(!found) {
        throw std::runtime_error("No available device found");
    }

    std::string version;
    bool userBootloader = false;
    {
        dai::DeviceBootloader bootloader(deviceInfo);
        version = bootloader.getVersion().toString();
        userBootloader = bootloader.isUserBootloader();
    }

    const std::string bootloaderType = userBootloader ? "User flashed bootloader" : "Factory flashed bootloader";

    std::cout << deviceInfo.toString() << '\n';
    std::cout << "Bootloader version: " << bootloaderType << ' ' << version << '\n';
    std::cout << "Embedded depthai bootloader version: " << dai::DeviceBootloader::getEmbeddedBootloaderVersion() << '\n';
    return 0;
}
