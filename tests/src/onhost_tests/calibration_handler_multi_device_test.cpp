#include <catch2/catch_all.hpp>
#include <filesystem>

#include "depthai/depthai.hpp"

namespace {

const dai::CoordinateFrame frameA{"deviceA", dai::CameraBoardSocket::CAM_A};
const dai::CoordinateFrame frameB{"deviceB", dai::CameraBoardSocket::CAM_A};

dai::Extrinsics makeExtrinsics(const dai::CoordinateFrame& to, float tx) {
    dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, dai::Point3f(tx, 0, 0), to.socket);
    extrinsics.setReferenceFrame(to);
    return extrinsics;
}

}  // namespace

TEST_CASE("CalibrationHandler stores extrinsics by device and socket") {
    dai::CalibrationHandler handler;
    handler.setExtrinsics(frameA.deviceId, frameA.socket, frameB.deviceId, frameB.socket, makeExtrinsics(frameB, 10.0f));

    const auto extrinsics = handler.getExtrinsics(frameA.deviceId, frameA.socket, frameB.deviceId, frameB.socket);
    REQUIRE(extrinsics.getReferenceFrame() == frameB);
    REQUIRE(extrinsics.toDeviceId == std::make_optional<std::string>("deviceB"));
    REQUIRE(extrinsics.getTransformationMatrix()[0][3] == Catch::Approx(10.0f));
    REQUIRE_THROWS_AS(handler.getExtrinsics(frameB.deviceId, frameB.socket, frameA.deviceId, frameA.socket), std::runtime_error);
}

TEST_CASE("CalibrationHandler converts explicitly stored extrinsics to the requested unit") {
    dai::CalibrationHandler handler;
    handler.setExtrinsics(frameA.deviceId, frameA.socket, frameB.deviceId, frameB.socket, makeExtrinsics(frameB, 10.0f));

    const auto matrix = handler.getExtrinsics(frameA.deviceId, frameA.socket, frameB.deviceId, frameB.socket, dai::LengthUnit::MILLIMETER)
                            .getTransformationMatrix(false, dai::LengthUnit::MILLIMETER);
    REQUIRE(matrix[0][3] == Catch::Approx(100.0f));
}

TEST_CASE("CalibrationHandler stores device extrinsics in EEPROM JSON") {
    dai::CalibrationHandler handler;
    handler.setExtrinsics(frameA.deviceId, frameA.socket, frameB.deviceId, frameB.socket, makeExtrinsics(frameB, 10.0f));

    const auto json = handler.eepromToJson();
    REQUIRE(json.at("devicesData").size() == 1);
    REQUIRE(handler.getEepromData().devicesData.at("deviceA").at(dai::CameraBoardSocket::CAM_A).getReferenceFrame() == frameB);

    const auto loaded = dai::CalibrationHandler::fromJson(json);
    REQUIRE(loaded.getExtrinsics(frameA.deviceId, frameA.socket, frameB.deviceId, frameB.socket).getTransformationMatrix()[0][3] == Catch::Approx(10.0f));
}
