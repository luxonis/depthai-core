#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

std::string toString(dai::UsbGeneration generation) {
    switch(generation) {
        case dai::UsbGeneration::USB_1_0:
            return "USB 1.0";
        case dai::UsbGeneration::USB_1_1:
            return "USB 1.1";
        case dai::UsbGeneration::USB_2_0:
            return "USB 2.0";
        case dai::UsbGeneration::USB_3_0:
            return "USB 3.0";
        case dai::UsbGeneration::USB_3_1:
            return "USB 3.1";
        case dai::UsbGeneration::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

void printMessages(const std::string& title, const std::unordered_map<std::string, std::string>& messages) {
    if(messages.empty()) {
        return;
    }

    std::vector<std::pair<std::string, std::string>> sortedMessages(messages.begin(), messages.end());
    std::sort(sortedMessages.begin(), sortedMessages.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    std::cout << title << ":" << std::endl;
    for(const auto& message : sortedMessages) {
        std::cout << "  " << message.first << ": " << message.second << std::endl;
    }
}

int main() {
    dai::HealthCheckConfig config;
    config.timeout = std::chrono::milliseconds(10000);
    config.checkUsbSpeed = true;
    config.measureBandwidth = true;
    config.verifyCameras = true;
    config.verifyIMU = true;
    config.verifyPowerSupply = true;

    dai::DeviceInfo deviceInfo;
    bool found = false;
    std::tie(found, deviceInfo) = dai::Device::getFirstAvailableDevice(false);
    if(!found) {
        std::cerr << "No DepthAI device found." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Device: " << deviceInfo.toString() << std::endl;
    std::cout << "Timeout: " << config.timeout.count() << " ms" << std::endl;
    std::cout << "Running health check..." << std::endl;

    const auto start = std::chrono::steady_clock::now();
    const auto metrics = dai::Device::performHealthCheck(deviceInfo, config);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);

    std::cout << std::endl;
    std::cout << "Health check completed in " << elapsed.count() << " ms" << std::endl;
    std::cout << "App running on device: " << (metrics.appRunningOnDevice ? "true" : "false") << std::endl;
    std::cout << "Device in setup mode: " << (metrics.inSetupMode ? "true" : "false") << std::endl;
    std::cout << "Udev rules set: " << (metrics.udevRulesSet ? "true" : "false") << std::endl;
    if(metrics.usbSpeed.has_value()) {
        std::cout << "USB speed: " << *metrics.usbSpeed << std::endl;
    } else {
        std::cout << "USB speed: not detected" << std::endl;
    }

    if(metrics.usbGeneration.has_value()) {
        std::cout << "USB generation: " << toString(*metrics.usbGeneration) << std::endl;
    } else {
        std::cout << "USB generation: not detected" << std::endl;
    }

    if(metrics.bandwidthMbps.has_value()) {
        std::cout << "Bandwidth: " << *metrics.bandwidthMbps << " Mbps" << std::endl;
    } else {
        std::cout << "Bandwidth: not measured" << std::endl;
    }

    std::cout << "Camera calibration: " << (metrics.cameraCalibration.has_value() ? (*metrics.cameraCalibration ? "pass" : "fail") : "not run") << std::endl;
    std::cout << "IMU calibration: " << (metrics.imuCalibration.has_value() ? (*metrics.imuCalibration ? "pass" : "fail") : "not run") << std::endl;
    std::cout << "Camera functionality: " << (metrics.cameraFunctionality.has_value() ? (*metrics.cameraFunctionality ? "pass" : "fail") : "not run")
              << std::endl;
    std::cout << "IMU functionality: " << (metrics.imuFunctionality.has_value() ? (*metrics.imuFunctionality ? "pass" : "fail") : "not run") << std::endl;
    std::cout << "Power supply: " << (metrics.powerSupplyFunctionality.has_value() ? (*metrics.powerSupplyFunctionality ? "pass" : "fail") : "not run")
              << std::endl;

    printMessages("Warnings", metrics.warnings);
    printMessages("Errors", metrics.errors);

    return 0;
}
