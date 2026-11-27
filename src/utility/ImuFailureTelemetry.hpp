#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace dai {
namespace utility {

/**
 * Properties of a depthai_imu_error event (error_type, error_message and error_code when the message has one),
 * or null when the device log message is not an IMU detection, initialization or firmware failure.
 */
nlohmann::json imuFailureTelemetry(const std::string& logMessage);

}  // namespace utility
}  // namespace dai
