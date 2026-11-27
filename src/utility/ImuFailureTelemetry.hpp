#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace dai {
namespace utility {

/**
 * Properties of a depthai_imu_error event (error_type, error_message and error_code when the message has one),
 * or null when the message is not an IMU detection, initialization or firmware failure of the IMU node.
 * nodeIdName is the name of the node that sent the message, such as "IMU(3)".
 */
nlohmann::json imuFailureTelemetry(const std::string& nodeIdName, const std::string& message);

}  // namespace utility
}  // namespace dai
