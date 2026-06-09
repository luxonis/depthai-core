#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
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

std::string toString(dai::HealthCheckResult result) {
    switch(result) {
        case dai::HealthCheckResult::PASS:
            return "pass";
        case dai::HealthCheckResult::FAIL:
            return "fail";
        case dai::HealthCheckResult::NOT_RUN:
        default:
            return "not run";
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
    config.checkUsbSpeed = true;
    config.measureBandwidth = true;
    config.verifyCameraFunctionality = true;
    config.verifyCameraCalibration = true;
    config.verifyImuFunctionality = true;
    config.verifyImuCalibration = true;
    config.verifyPowerSupply = true;
    config.powerSupplyCheckDuration = std::chrono::milliseconds(10000);

    dai::DeviceInfo deviceInfo;
    bool found = false;
    std::tie(found, deviceInfo) = dai::Device::getFirstAvailableDevice(false);
    if(!found) {
        std::cerr << "No DepthAI device found." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Device: " << deviceInfo.toString() << std::endl;
    std::cout << "Power supply check duration: " << config.powerSupplyCheckDuration.count() << " ms" << std::endl;
    std::cout << "Running health check..." << std::endl;

    const auto start = std::chrono::steady_clock::now();
    const auto metrics = dai::Device::performHealthCheck(deviceInfo, config);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);

    std::cout << std::endl;
    std::cout << "Health check completed in " << elapsed.count() << " ms" << std::endl;
    std::cout << "App running on device: " << (metrics.appRunningOnDevice ? "true" : "false") << std::endl;
    std::cout << "Device in setup mode: " << (metrics.inSetupMode ? "true" : "false") << std::endl;
    std::cout << "Udev rules set: " << (metrics.udevRulesSet ? "true" : "false") << std::endl;
    std::cout << "USB generation: " << toString(metrics.usbGeneration) << std::endl;
    std::cout << "Bandwidth: " << metrics.bandwidthMbps << " Mbps" << std::endl;
    std::cout << "Camera calibration: " << toString(metrics.cameraCalibration) << std::endl;
    std::cout << "IMU calibration: " << toString(metrics.imuCalibration) << std::endl;
    std::cout << "Camera functionality: " << toString(metrics.cameraFunctionality) << std::endl;
    std::cout << "IMU functionality: " << toString(metrics.imuFunctionality) << std::endl;
    std::cout << "Power supply: " << toString(metrics.powerSupplyFunctionality) << std::endl;

    printMessages("Warnings", metrics.warnings);
    printMessages("Errors", metrics.errors);

    return 0;
}
