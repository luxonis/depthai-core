#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "depthai/common/UsbSpeed.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

/**
 * USB generation reported by the device health check.
 */
enum class UsbGeneration : std::uint8_t { UNKNOWN, USB_1_0, USB_1_1, USB_2_0, USB_3_0, USB_3_1 };

/**
 * Configures which device health-check steps should run.
 */
struct HealthCheckConfig {
    bool checkUsbSpeed = true;
    bool measureBandwidth = true;
    bool verifyCameras = true;
    bool verifyIMU = true;
    bool verifyMaxPower = true;

    std::chrono::milliseconds timeout{std::chrono::seconds(10)};
};

/**
 * Device health-check results.
 *
 * Optional fields are empty when the corresponding check was disabled or could
 * not be run because an earlier prerequisite failed.
 */
struct HealthCheckMetrics {
    std::optional<UsbSpeed> usbSpeed;
    std::optional<UsbGeneration> usbGeneration;
    std::optional<float> bandwidthMbps;
    std::optional<bool> cameraFunctionality;
    std::optional<bool> cameraCalibration;
    std::optional<bool> imuFunctionality;
    std::optional<bool> imuCalibration;
    std::optional<bool> maxPowerFunctionality;

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
