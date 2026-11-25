// The RVC4 firmware's B-spline calibration store (device/rvc4/src/BsplineCalibrationStore.hpp), on a temporary
// directory instead of /persist/calibration.
#include <catch2/catch_all.hpp>
#include <filesystem>
#include <fstream>

#include "BsplineCalibrationStore.hpp"

namespace {
dai::BsplineStereoCalibration model() {
    dai::BsplineCameraCalibration camera;
    camera.nx = camera.ny = 4;
    camera.fovXDegrees = 80;
    camera.intrinsics = std::vector<double>(36, 0.0);
    camera.intrinsics[0] = camera.intrinsics[1] = 400;
    dai::BsplineStereoCalibration result;
    result.deviceId = "device";
    result.calibrationId = "id";
    result.leftSocket = dai::CameraBoardSocket::CAM_B;
    result.rightSocket = dai::CameraBoardSocket::CAM_C;
    result.width = 1280;
    result.height = 800;
    result.left = result.right = camera;
    result.rotationLeft = result.rotationRight = result.leftToRightRotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    result.leftToRightTranslationCm = {-7.5, 0, 0};
    result.baselineCm = 7.5;
    result.virtualIntrinsics = {400, 400, 640, 400};
    return result;
}
}  // namespace

TEST_CASE("B-spline calibration store keeps the model bound to its EEPROM data and device") {
    const auto directory = std::filesystem::temp_directory_path() / "depthai_bspline_store_test" / "calibration";
    std::filesystem::remove_all(directory.parent_path());
    const dai::device::BsplineCalibrationStore store(directory);
    const std::vector<uint8_t> legacy(512, 7), otherLegacy(512, 8);

    REQUIRE_FALSE(store.load(false, legacy, "device").has_value());
    store.store(false, legacy, model(), "device");
    const auto loaded = store.load(false, legacy, "device");
    REQUIRE(loaded.has_value());
    CHECK(loaded->baselineCm == 7.5);
    CHECK(loaded->left.intrinsics == model().left.intrinsics);
    CHECK_FALSE(store.load(true, legacy, "device").has_value());  // user and factory areas are separate files

    CHECK_THROWS(store.load(false, otherLegacy, "device"));  // the EEPROM changed after the model was stored
    CHECK_THROWS(store.load(false, legacy, "other-device"));
    CHECK_THROWS(store.store(false, legacy, model(), "other-device"));

    SECTION("A corrupted file is rejected") {
        const auto file = directory / "bspline-user.cbor";
        std::fstream stream(file, std::ios::in | std::ios::out | std::ios::binary);
        const auto offset = static_cast<std::streamoff>(std::filesystem::file_size(file) - 40);
        stream.seekg(offset);
        const char byte = static_cast<char>(stream.get());
        stream.seekp(offset);
        stream.put(static_cast<char>(byte ^ 0x55));
        stream.close();
        CHECK_THROWS(store.load(false, legacy, "device"));
    }
    SECTION("Removing the model deletes the file, also twice") {
        store.store(false, legacy, std::nullopt, "device");
        store.store(false, legacy, std::nullopt, "device");
        CHECK_FALSE(std::filesystem::exists(directory / "bspline-user.cbor"));
        CHECK_FALSE(store.load(false, legacy, "device").has_value());
    }
    std::filesystem::remove_all(directory.parent_path());
}
