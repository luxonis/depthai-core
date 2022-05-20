#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.name << std::endl;
        dai::DeviceBootloader bl(info);
        std::cout << "Version: " << bl.getVersion().toString() << std::endl;

        for(const auto& mem : {dai::DeviceBootloader::Memory::FLASH, dai::DeviceBootloader::Memory::EMMC}) {
            std::cout << std::endl;
            auto memoryInfo = bl.getMemoryInfo(mem);
            if(memoryInfo.available) {
                std::cout << "Memory '" << mem << "' size: " << memoryInfo.size << ", info: " << memoryInfo.info << std::endl;

                auto appInfo = bl.readApplicationInfo(mem);
                std::cout << "Application, flashed: " << appInfo.hasApplication << " firmware version: " << appInfo.firmwareVersion
                          << " application name: " << appInfo.applicationName << std::endl;
            } else {
                std::cout << "Memory '" << mem << "' not available..." << std::endl;
            }
        }
    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}