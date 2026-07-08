#include <chrono>
#include <cstdlib>
#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    dai::HealthCheckConfig config;
    config.powerSupplyCheckDuration = std::chrono::milliseconds(20000);

    const auto devices = dai::Device::getAllConnectedDevices();
    if(devices.empty()) {
        std::cerr << "No DepthAI device found." << std::endl;
        return EXIT_FAILURE;
    }
    const auto& deviceInfo = devices.front();

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
