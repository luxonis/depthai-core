#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.name << std::endl;
        dai::DeviceBootloader bl(info);
        std::cout << "Version: " << bl.getVersion().toString() << std::endl;

        bool ok;
        std::string err;
        dai::DeviceBootloader::ApplicationInfo info;
        std::tie(ok, err, info) = bl.readApplicationInfo();
        if(ok) {
            std::cout << "Application, flashed: " << info.hasApplication << " firmware version: " << info.firmwareVersion
                      << " application name: " << info.applicationName << std::endl;
        } else {
            std::cout << "Error reading application infomation: " << err << std::endl;
        }

        for(const auto& mem : {dai::DeviceBootloader::Memory::FLASH, dai::DeviceBootloader::Memory::EMMC}) {
            dai::DeviceBootloader::MemoryInfo info;
            std::tie(ok, err, info) = bl.getMemoryInfo(mem);
            if(ok) {
                std::cout << "Memory size: " << info.size << ", info: " << info.info << std::endl;
            } else {
                std::cout << "Error retrieving memory information: " << err << std::endl;
            }
        }
    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}