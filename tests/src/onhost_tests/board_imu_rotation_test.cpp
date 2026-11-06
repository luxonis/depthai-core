#include <catch2/catch_all.hpp>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>

#include "depthai/device/CalibrationHandler.hpp"

TEST_CASE("Every board IMU rotation can be stored in device calibration", "[imu][boards]") {
    const std::filesystem::path boardsPath = DEPTHAI_BOARDS_PATH;
    REQUIRE(std::filesystem::is_directory(boardsPath));

    size_t boardCount = 0;
    size_t rotationCount = 0;
    for(const auto& entry : std::filesystem::directory_iterator(boardsPath)) {
        if(!entry.is_regular_file() || entry.path().extension() != ".json") continue;
        if(entry.path().filename() == "RAE-A-B-C.json") continue;
        ++boardCount;
        CAPTURE(entry.path().string());

        std::ifstream stream(entry.path());
        REQUIRE(stream.is_open());
        const auto board = nlohmann::json::parse(stream);
        const auto& config = board.at("board_config");
        if(!config.contains("imuExtrinsics")) continue;

        const auto& sensors = config.at("imuExtrinsics").at("sensors");
        for(auto sensor = sensors.begin(); sensor != sensors.end(); ++sensor) {
            CAPTURE(sensor.key());
            REQUIRE(sensor.value().contains("extrinsics"));
            const auto& extrinsics = sensor.value().at("extrinsics");
            const bool hasRotationMatrix = extrinsics.contains("rotationMatrix");
            CHECK(hasRotationMatrix);
            if(!hasRotationMatrix) continue;
            const auto rotation = extrinsics.at("rotationMatrix").get<std::vector<std::vector<float>>>();
            ++rotationCount;

            dai::CalibrationHandler calibration;
            REQUIRE_NOTHROW(calibration.setImuExtrinsics(dai::CameraBoardSocket::CAM_A, rotation, {0, 0, 0}, {0, 0, 0}));

            // EEPROM serialization is the representation sent to the device.
            const nlohmann::json uploaded = calibration.getEepromData();
            const auto restored = dai::CalibrationHandler::fromJson(uploaded);
            const auto restoredData = restored.getEepromData();
            const auto& stored = restoredData.imuExtrinsics.rotationMatrix;
            REQUIRE(stored.size() == 3);
            for(size_t row = 0; row < 3; ++row) {
                REQUIRE(stored[row].size() == 3);
                for(size_t column = 0; column < 3; ++column) {
                    REQUIRE(stored[row][column] == Catch::Approx(rotation[row][column]).margin(1e-6));
                }
            }
        }
    }

    REQUIRE(boardCount > 0);
    REQUIRE(rotationCount > 0);
}
