#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

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
    bool checkUsbGeneration;
    bool measureBandwidth;
    bool verifyCameraFunctionality;
    bool verifyCameraCalibration;
    bool verifyImuFunctionality;
    bool verifyImuCalibration;
    bool verifyPowerSupply;

    std::chrono::milliseconds powerSupplyCheckDuration;

    explicit HealthCheckConfig(bool checkUsbGeneration = true,
                               bool measureBandwidth = true,
                               bool verifyCameraFunctionality = true,
                               bool verifyCameraCalibration = true,
                               bool verifyImuFunctionality = true,
                               bool verifyImuCalibration = true,
                               bool verifyPowerSupply = true,
                               std::chrono::milliseconds powerSupplyCheckDuration = std::chrono::seconds(20))
        : checkUsbGeneration(checkUsbGeneration),
          measureBandwidth(measureBandwidth),
          verifyCameraFunctionality(verifyCameraFunctionality),
          verifyCameraCalibration(verifyCameraCalibration),
          verifyImuFunctionality(verifyImuFunctionality),
          verifyImuCalibration(verifyImuCalibration),
          verifyPowerSupply(verifyPowerSupply),
          powerSupplyCheckDuration(powerSupplyCheckDuration) {}
};

/**
 * Health check issue type
 */
enum class HealthCheckIssueType : std::uint8_t {
    Warning,
    Error,
};

/**
 * Health check issue stage
 */
enum class HealthCheckIssueStage : std::uint8_t {
    Connection,
    DeviceAvailability,
    UsbGeneration,
    Bandwidth,
    CameraFunctionality,
    CameraCalibration,
    ImuFunctionality,
    ImuCalibration,
    PowerSupply,
};

/**
 * Health check issue, which contains the details of the issue
 */
struct HealthCheckIssue {
    HealthCheckIssueType type;
    HealthCheckIssueStage stage;
    std::string message;

    HealthCheckIssue() = delete;
    HealthCheckIssue(HealthCheckIssueType type, HealthCheckIssueStage stage, std::string message) : type(type), stage(stage), message(std::move(message)) {}
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

    bool deviceInUse = false;
    bool deviceInSetupMode = false;
    bool missingUdevRules = false;

    std::vector<HealthCheckIssue> issues;

    std::string toString() const;
};

class DeviceHealthCheck {
    friend class DeviceBase;

   public:
    DeviceHealthCheck() = delete;

    /**
     * Performs all configured health-check steps and aggregates the results.
     */
   private:
    static HealthCheckMetrics run(const DeviceInfo& devInfo, const HealthCheckConfig& config);
};

}  // namespace dai
