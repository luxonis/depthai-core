#include <argparse/argparse.hpp>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    argparse::ArgumentParser program("check_device_gpu", "1.0.0");
    program.add_description("Print basic device GPU availability info.");
    program.add_argument("--device", "-d").default_value(std::string("")).help("Device IP address / device ID (default: auto-discover)");

    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << '\n';
        std::cerr << program;
        return EXIT_FAILURE;
    }

    const auto deviceArg = program.get<std::string>("--device");
    dai::Device device = deviceArg.empty() ? dai::Device() : dai::Device(deviceArg);

    std::cout << "Product: " << device.getProductName() << std::endl;
    std::cout << "Platform: " << device.getPlatformAsString() << std::endl;
    std::cout << "GPU available: " << (device.hasGPU() ? "true" : "false") << std::endl;
    return 0;
}
