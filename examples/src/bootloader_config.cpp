#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    // Options
    bool usage = false, read = true, clear = false;
    std::string path = "";
    if(argc >= 2) {
        std::string op = argv[1];
        if(op == "read") {
            read = true;
        } else if(op == "flash") {
            read = false;
            if(argc >= 3) {
                path = argv[2];
            }
        } else if(op == "clear") {
            clear = true;
            read = false;
        } else if(op == "clear") {
            clear = true;
            read = false;
        } else {
            usage = true;
        }
    } else {
        usage = true;
    }
    if(usage) {
        std::cout << "Usage: " << argv[0] << " [read/flash/clear] [flash: path/to/config/json]" << std::endl;
        return -1;
    }

    // DeviceBootloader configuration
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.desc.name << std::endl;
        dai::DeviceBootloader bl(info);

        if(read) {
            std::cout << "Current flashed configuration\n" << bl.readConfigData().dump(4) << std::endl;
        } else {
            bool success;
            std::string error;
            if(clear) {
                std::tie(success, error) = bl.flashConfigClear();
            } else {
                if(path.empty()) {
                    std::tie(success, error) = bl.flashConfig(dai::DeviceBootloader::Config{});
                } else {
                    std::tie(success, error) = bl.flashConfigFile(path);
                }
            }
            if(success) {
                std::cout << "Successfully flashed bootloader configuration\n";
            } else {
                std::cout << "Error flashing bootloader configuration: " << error;
            }
        }

    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}