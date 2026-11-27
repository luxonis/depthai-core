#include <catch2/catch_all.hpp>
#include <string>

#include "utility/ImuFailureTelemetry.hpp"

using dai::utility::imuFailureTelemetry;

// Messages as the RVC2 firmware and the RVC4 IMU node send them: as log messages of the "IMU(<id>)" logger,
// and the RVC2 "not detected" one as the pipeline build error, which starts with "IMU(<id>)"
TEST_CASE("IMU failure log messages map to depthai_imu_error properties") {
    struct Case {
        std::string message;
        std::string errorType;
        std::string errorCode;
    };
    const auto testCase = GENERATE(
        Case{"IMU(3) - IMU invalid settings!: IMU not detected. Your board doesn't have IMU or requires firmware update (Set "
             "imu.enableFirmwareUpdate(True) explicitly.). 0, 485",
             "imu_not_detected",
             "0, 485"},
        Case{"Node threw exception, stopping the node. Exception message: IMU not detected. Your board doesn't have IMU.", "imu_not_detected", ""},
        Case{"Node threw exception, stopping the node. Exception message: IMU not detected. Your board doesn't have IMU. -5, 607",
             "imu_not_detected",
             "-5, 607"},
        Case{"IMU sensor not detected. Your board does not requested sensor type 8", "imu_sensor_not_detected", ""},
        Case{"Failed to get sensor 1 attributes", "imu_sensor_attributes_failed", ""},
        Case{"Failed to start sensor 4 streaming", "imu_sensor_start_failed", ""},
        Case{"Node threw exception, stopping the node. Exception message: Sensor direct channel: dai_device_sensor_lookup failed: not found",
             "imu_sensor_channel_failed",
             ""},
        Case{"IMU firmware version detected: 3.2.13 is not supported. Latest supported: 3.9.7. Use 'imu.enableFirmwareUpdate(True)' once to update firmware "
             "to latest.",
             "imu_fw_update_required",
             ""},
        Case{"Performing IMU firmware update, do NOT disconnect the device.", "imu_fw_update_in_progress", ""},
        Case{"IMU firmware update failed. Your board likely doesn't have IMU!", "imu_fw_update_failed", ""},
        Case{"Error reading IMU FW version!", "imu_fw_version_read_failed", ""},
        Case{"IMU driver failed with error code -3. Your board likely doesn't have IMU!", "imu_driver_failed", "-3"},
        Case{"IMU driver error 2 -1", "imu_driver_error", "2 -1"},
        Case{"Failed to initialize BMI270! Error code: '-2'", "imu_init_failed", "-2"});
    CAPTURE(testCase.message);

    const auto properties = imuFailureTelemetry("IMU(3)", testCase.message);
    REQUIRE(properties.is_object());
    CHECK(properties.at("error_type") == testCase.errorType);
    CHECK(properties.at("error_message") == testCase.message);
    if(testCase.errorCode.empty()) {
        CHECK_FALSE(properties.contains("error_code"));
    } else {
        CHECK(properties.at("error_code") == testCase.errorCode);
    }
}

TEST_CASE("Other device log messages are not IMU failures") {
    CHECK(imuFailureTelemetry("IMU(3)", "IMU firmware update status: 42%").is_null());
    CHECK(imuFailureTelemetry("IMU(3)", "IMU firmware update successful!").is_null());
    // Other nodes, such as a Script node, can print the same words
    CHECK(imuFailureTelemetry("Script(5)", "retry 3 of 5 after IMU driver error").is_null());
}

TEST_CASE("A long digit run after an IMU failure does not overflow the stack") {
    const auto properties = imuFailureTelemetry("IMU(3)", "IMU driver error " + std::string(100000, '7'));
    REQUIRE(properties.is_object());
    CHECK(properties.at("error_type") == "imu_driver_error");
}
