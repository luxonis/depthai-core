#include <chrono>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();
    std::string path;

    if(res) {
        std::cout << "Found device with name: " << info.desc.name << std::endl;
        dai::DeviceBootloader bl(info, path);
        std::cout << "Version: " << bl.getVersion().toString() << std::endl;
    } else {
        std::cout << "Didn't find any devices" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    return 0;
}