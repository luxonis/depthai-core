#include "utility/ImuFailureTelemetry.hpp"

#include <cstring>
#include <regex>

namespace dai {
namespace utility {

nlohmann::json imuFailureTelemetry(const std::string& nodeIdName, const std::string& message) {
    // The IMU node logs as "IMU(<id>)" on RVC2 and RVC4; other nodes can print the same words
    if(nodeIdName.rfind("IMU(", 0) != 0) {
        return nullptr;
    }
    // Messages of the RVC2 firmware IMU node and of the RVC4 IMU node: a change to the messages in
    // device/rvc4/src/pipeline/node/IMU/rvc4/IMU.cpp must change this table too
    struct ImuFailure {
        const char* text;
        const char* errorType;
        bool hasErrorCode;
    };
    static constexpr ImuFailure failures[] = {
        {"IMU not detected", "imu_not_detected", true},
        {"IMU sensor not detected", "imu_sensor_not_detected", false},
        {"Failed to get sensor", "imu_sensor_attributes_failed", false},
        {"Failed to start sensor", "imu_sensor_start_failed", false},
        {"Sensor direct channel:", "imu_sensor_channel_failed", false},
        {"IMU firmware version detected:", "imu_fw_update_required", false},
        {"Performing IMU firmware update", "imu_fw_update_in_progress", false},
        {"IMU firmware update failed", "imu_fw_update_failed", false},
        {"Error reading IMU FW version", "imu_fw_version_read_failed", false},
        {"IMU driver failed with error code", "imu_driver_failed", true},
        {"IMU driver error", "imu_driver_error", true},
        {"Failed to initialize BMI270", "imu_init_failed", true},
    };
    // The driver error code, and the source line when the firmware adds it: "0, 485"
    static const std::regex errorCode(R"(-?\d+(?:,? -?\d+)?)");
    // std::regex recurses per character: a bounded search keeps a long digit run from overflowing the stack
    constexpr std::size_t maxErrorCodeSearch = 256;

    for(const auto& failure : failures) {
        const auto pos = message.find(failure.text);
        if(pos == std::string::npos) {
            continue;
        }
        nlohmann::json properties{{"error_type", failure.errorType}, {"error_message", message}};
        if(failure.hasErrorCode) {
            const auto afterText = message.substr(pos + std::strlen(failure.text), maxErrorCodeSearch);
            std::smatch code;
            if(std::regex_search(afterText, code, errorCode)) {
                properties["error_code"] = code.str();
            }
        }
        return properties;
    }
    return nullptr;
}

}  // namespace utility
}  // namespace dai
