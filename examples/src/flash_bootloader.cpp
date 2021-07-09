#include <chrono>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    using namespace std::chrono;

    dai::DeviceBootloader::Type blType = dai::DeviceBootloader::Type::USB;
    if(argc > 1) {
        if(std::string(argv[1]) == "usb") {
            blType = dai::DeviceBootloader::Type::USB;
        } else if(std::string(argv[1]) == "eth") {
            blType = dai::DeviceBootloader::Type::NETWORK;
        } else {
            std::cout << "Specify either 'usb' or 'eth' bootloader type\n";
            return 0;
        }
    } else {
        std::cout << "Usage: " << argv[0] << " <usb/eth>\n";
        return 0;
    }

    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    dai::DeviceBootloader bl(info);
    auto progress = [](float p) { std::cout << "Flashing Progress..." << p * 100 << "%" << std::endl; };

    std::string message;
    auto t1 = steady_clock::now();
    std::tie(res, message) = bl.flashBootloader(dai::DeviceBootloader::Memory::FLASH, blType, progress);
    if(res) {
        std::cout << "Flashing successful. Took " << duration_cast<milliseconds>(steady_clock::now() - t1).count() << "ms" << std::endl;
    } else {
        std::cout << "Flashing failed: " << message << std::endl;
    }
    return 0;
}