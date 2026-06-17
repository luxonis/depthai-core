#include <chrono>
#include <cstdlib>
#include <iostream>
#include <tuple>

#include "depthai/depthai.hpp"

int main() {
    dai::HealthCheckConfig config;
    config.checkUsbGeneration = true;
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
    std::cout << metrics.toString() << std::endl;
    return 0;
}
