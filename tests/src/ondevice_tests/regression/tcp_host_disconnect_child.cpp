#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc != 2) return EXIT_FAILURE;
    try {
        dai::Device device{dai::DeviceInfo(argv[1])};
        device.setLogLevel(dai::LogLevel::INFO);
        device.setSystemInformationLoggingRate(1.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Terminate without running Device's destructor, as when the host is killed.
        std::_Exit(EXIT_SUCCESS);
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
