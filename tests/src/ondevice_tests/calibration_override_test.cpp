#include <catch2/catch_all.hpp>
#include <optional>

#include "depthai/depthai.hpp"

namespace {

dai::CalibrationHandler calibrationWithFocal(float focal) {
    dai::CalibrationHandler calibration;
    calibration.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, {{focal, 0, 320}, {0, 810, 240}, {0, 0, 1}}, 640, 480);
    return calibration;
}

float focal(const dai::CalibrationHandler& calibration) {
    return calibration.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A)[0][0];
}

}  // namespace

TEST_CASE("Session calibration override leaves stored calibration untouched") {
    dai::Device device;
    const auto active = device.getCalibration().eepromToJson();
    const auto stored = device.readCalibration().eepromToJson();

    device.setCalibration(calibrationWithFocal(1000));
    REQUIRE(focal(device.getCalibration()) == 1000);
    REQUIRE(device.readCalibration().eepromToJson() == stored);

    device.setCalibration(dai::CalibrationHandler());
    REQUIRE_FALSE(device.getCalibration().hasCalibrationData());
    REQUIRE(device.readCalibration().eepromToJson() == stored);

    device.setCalibration(std::nullopt);
    REQUIRE(device.getCalibration().eepromToJson() == active);
}
