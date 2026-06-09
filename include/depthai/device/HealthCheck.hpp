#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

/**
 * USB generation reported by the device health check.
 */
enum class UsbGeneration : std::uint8_t { UNKNOWN, USB_1_0, USB_1_1, USB_2_0, USB_3_0, USB_3_1 };

/**
 * Health check result reported by the device health check.
 */
enum class HealthCheckResult : std::uint8_t { NOT_RUN, PASS, FAIL };

/**
 * Configures which device health-check steps should run.
 */
struct HealthCheckConfig {
    bool checkUsbSpeed = true;
    bool measureBandwidth = true;
    bool verifyCameraFunctionality = true;
    bool verifyCameraCalibration = true;
    bool verifyImuFunctionality = true;
    bool verifyImuCalibration = true;
    bool verifyPowerSupply = true;

    std::chrono::milliseconds powerSupplyCheckDuration{std::chrono::seconds(20)};
};

/**
 * Device health-check results.
 */
struct HealthCheckMetrics {
    UsbGeneration usbGeneration = UsbGeneration::UNKNOWN;
    float bandwidthMbps = 0.0f;
    HealthCheckResult cameraFunctionality = HealthCheckResult::NOT_RUN;
    HealthCheckResult cameraCalibration = HealthCheckResult::NOT_RUN;
    HealthCheckResult imuFunctionality = HealthCheckResult::NOT_RUN;
    HealthCheckResult imuCalibration = HealthCheckResult::NOT_RUN;
    HealthCheckResult powerSupplyFunctionality = HealthCheckResult::NOT_RUN;

    bool appRunningOnDevice = false;
    bool inSetupMode = false;
    bool udevRulesSet = false;

    std::unordered_map<std::string, std::string> errors;
    std::unordered_map<std::string, std::string> warnings;
};

class DeviceHealthCheck {
    friend class DeviceBase;

   public:
    DeviceHealthCheck() = delete;

    /**
     * Performs all configured health-check steps and aggregates the results.
     */
   private:
    static HealthCheckMetrics run(const DeviceInfo& devInfo, const HealthCheckConfig& config = {});
};

}  // namespace dai
