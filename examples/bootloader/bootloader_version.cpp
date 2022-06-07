#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.name << std::endl;
        dai::DeviceBootloader bl(info);
        std::cout << "Version: " << bl.getVersion().toString() << std::endl;
    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}