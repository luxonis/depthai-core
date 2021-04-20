#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();
    std::string path;
    if(argc = 2) {
        path = std::string(argv[1]);
    } else {
        throw std::runtime_error("No path specified for boot config file");
    }

    dai::DeviceBootloader bl(info, path);
    auto progress = [](float p) { std::cout << "Flashing Progress..." << p * 100 << "%" << std::endl; };

    bl.flashBootloader(progress);
    return 0;
}