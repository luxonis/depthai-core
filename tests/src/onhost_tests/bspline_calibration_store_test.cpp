// The RVC4 firmware's B-spline calibration store (device/rvc4/src/BsplineCalibrationStore.hpp), on a temporary
// directory instead of /persist/calibration.
#include <catch2/catch_all.hpp>
#include <filesystem>
#include <fstream>

#include "BsplineCalibrationStore.hpp"

namespace {
dai::device::BsplineCalibrationStore::Lenses lenses() {
    dai::BsplineCameraCalibration lens;
    lens.width = 1280;
    lens.height = 800;
    lens.nx = lens.ny = 4;
    lens.fovXDegrees = 80;
    lens.intrinsics = std::vector<double>(36, 0.0);
    lens.intrinsics[0] = lens.intrinsics[1] = 400;
    return {{dai::CameraBoardSocket::CAM_B, lens}, {dai::CameraBoardSocket::CAM_C, lens}};
}
}  // namespace

TEST_CASE("B-spline calibration store keeps the model bound to its EEPROM data and device") {
    const auto directory = std::filesystem::temp_directory_path() / "depthai_bspline_store_test" / "calibration";
    std::filesystem::remove_all(directory.parent_path());
    const dai::device::BsplineCalibrationStore store(directory);
    const std::vector<uint8_t> legacy(512, 7), otherLegacy(512, 8);

    REQUIRE(store.load(false, legacy, "device").empty());
    store.store(false, legacy, lenses(), "device");
    const auto loaded = store.load(false, legacy, "device");
    REQUIRE(loaded.size() == 2);
    CHECK(loaded.at(dai::CameraBoardSocket::CAM_C).intrinsics == lenses().at(dai::CameraBoardSocket::CAM_C).intrinsics);
    CHECK(loaded.at(dai::CameraBoardSocket::CAM_B).width == 1280);
    CHECK(store.load(true, legacy, "device").empty());  // user and factory areas are separate files

    CHECK_THROWS(store.load(false, otherLegacy, "device"));  // the EEPROM changed after the model was stored
    CHECK_THROWS(store.load(false, legacy, "other-device"));

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
        store.store(false, legacy, {}, "device");
        store.store(false, legacy, {}, "device");
        CHECK_FALSE(std::filesystem::exists(directory / "bspline-user.cbor"));
        CHECK(store.load(false, legacy, "device").empty());
    }
    std::filesystem::remove_all(directory.parent_path());
}
