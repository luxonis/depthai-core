#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Connect to device and start pipeline
    dai::Device device;
    auto crashDump = device.getCrashDump();
    if(crashDump.crashReports.empty()) {
        std::cout << "There was no crash dump found on your device!" << std::endl;
    } else {
        auto json = crashDump.serializeToJson();
        std::cout << json << std::endl;

        dai::Path destPath = "crashDump.json"; 
        std::ofstream ob(destPath);
        ob << std::setw(4) << json << std::endl;
    }

    return 0;
}