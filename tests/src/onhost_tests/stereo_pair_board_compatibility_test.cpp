#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "../../../src/device/StereoPairUtils.hpp"

namespace {

using Matrix = std::vector<std::vector<float>>;
using Socket = dai::CameraBoardSocket;

constexpr float pi = 3.14159265358979323846f;

Socket parseSocket(const std::string& name) {
    static const std::unordered_map<std::string, Socket> sockets = {{"CAM_A", Socket::CAM_A},
                                                                    {"CAM_B", Socket::CAM_B},
                                                                    {"CAM_C", Socket::CAM_C},
                                                                    {"CAM_D", Socket::CAM_D},
                                                                    {"CAM_E", Socket::CAM_E},
                                                                    {"CAM_F", Socket::CAM_F},
                                                                    {"CAM_G", Socket::CAM_G},
                                                                    {"CAM_H", Socket::CAM_H},
                                                                    {"CAM_I", Socket::CAM_I},
                                                                    {"CAM_J", Socket::CAM_J}};
    return sockets.at(name);
}

Matrix multiply(const Matrix& first, const Matrix& second) {
    Matrix result(3, std::vector<float>(3, 0.0f));
    for(size_t row = 0; row < 3; ++row) {
        for(size_t column = 0; column < 3; ++column) {
            for(size_t index = 0; index < 3; ++index) result[row][column] += first[row][index] * second[index][column];
        }
    }
    return result;
}

Matrix parseRotation(const nlohmann::json& extrinsics) {
    if(extrinsics.contains("rotationMatrix")) return extrinsics.at("rotationMatrix").get<Matrix>();

    const auto rotation = extrinsics.value("rotation", nlohmann::json::object());
    const float roll = rotation.value("r", 0.0f) * pi / 180.0f;
    const float pitch = rotation.value("p", 0.0f) * pi / 180.0f;
    const float yaw = rotation.value("y", 0.0f) * pi / 180.0f;
    const Matrix rotateX = {{1, 0, 0}, {0, std::cos(roll), -std::sin(roll)}, {0, std::sin(roll), std::cos(roll)}};
    const Matrix rotateY = {{std::cos(pitch), 0, std::sin(pitch)}, {0, 1, 0}, {-std::sin(pitch), 0, std::cos(pitch)}};
    const Matrix rotateZ = {{std::cos(yaw), -std::sin(yaw), 0}, {std::sin(yaw), std::cos(yaw), 0}, {0, 0, 1}};
    return multiply(multiply(rotateZ, rotateY), rotateX);
}

std::vector<float> parseTranslation(const nlohmann::json& extrinsics) {
    const auto translation = extrinsics.value("specTranslation", nlohmann::json::object());
    return {translation.value("x", 0.0f), translation.value("y", 0.0f), translation.value("z", 0.0f)};
}

dai::CameraSensorType parseSensorType(const std::string& type) {
    if(type == "mono") return dai::CameraSensorType::MONO;
    if(type == "color") return dai::CameraSensorType::COLOR;
    return dai::CameraSensorType::TOF;
}

struct BoardFixture {
    dai::CalibrationHandler calibration;
    std::vector<dai::CameraFeatures> features;
};

BoardFixture makeBoardFixture(const nlohmann::json& board) {
    BoardFixture fixture;
    const Matrix intrinsics = {{1000, 0, 640}, {0, 1000, 400}, {0, 0, 1}};
    const Matrix identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const std::vector<float> zero = {0, 0, 0};

    for(const auto& [socketName, camera] : board.at("cameras").items()) {
        const auto socket = parseSocket(socketName);
        fixture.calibration.setCameraIntrinsics(socket, intrinsics, 1280, 800);

        dai::CameraFeatures feature;
        feature.socket = socket;
        feature.sensorName = camera.value("type", "unknown");
        feature.width = 1280;
        feature.height = 800;
        feature.supportedTypes = {parseSensorType(camera.value("type", "unknown"))};
        fixture.features.push_back(feature);
    }

    for(const auto& [socketName, camera] : board.at("cameras").items()) {
        const auto socket = parseSocket(socketName);
        if(camera.contains("extrinsics") && camera.at("extrinsics").contains("to_cam")
           && board.at("cameras").contains(camera.at("extrinsics").at("to_cam").get<std::string>())) {
            const auto& extrinsics = camera.at("extrinsics");
            const auto translation = parseTranslation(extrinsics);
            fixture.calibration.setCameraExtrinsics(socket, parseSocket(extrinsics.at("to_cam")), parseRotation(extrinsics), translation, translation);
        } else {
            fixture.calibration.setCameraExtrinsics(socket, Socket::AUTO, identity, zero, zero);
        }
    }
    return fixture;
}

}  // namespace

TEST_CASE("DepthAI board stereo configurations agree with getStereoPairs ordering", "[stereo-pair-ordering][boards]") {
    const char* boardsPathOverride = std::getenv("DEPTHAI_BOARDS_PATH");
    const std::filesystem::path boardsPath = boardsPathOverride != nullptr ? boardsPathOverride : DEPTHAI_BOARDS_PATH;
    REQUIRE(std::filesystem::is_directory(boardsPath));

    size_t testedBoards = 0;
    for(const auto& entry : std::filesystem::directory_iterator(boardsPath)) {
        if(entry.path().extension() != ".json") continue;

        std::ifstream stream(entry.path());
        const auto board = nlohmann::json::parse(stream).at("board_config");
        if(!board.contains("stereo_config")) continue;

        const auto& stereo = board.at("stereo_config");
        const auto leftName = stereo.at("left_cam").get<std::string>();
        const auto rightName = stereo.at("right_cam").get<std::string>();
        const auto& cameras = board.at("cameras");
        const auto leftType = cameras.at(leftName).value("type", "unknown");
        const auto rightType = cameras.at(rightName).value("type", "unknown");
        if(leftType != rightType || (leftType != "mono" && leftType != "color")) continue;

        DYNAMIC_SECTION(entry.path().filename().string()) {
            const auto fixture = makeBoardFixture(board);
            const auto pairs = dai::detail::StereoPairCalculator::find(fixture.calibration, fixture.features);
            const auto expected = std::find_if(pairs.begin(), pairs.end(), [&](const dai::StereoPair& pair) {
                return (pair.left == parseSocket(leftName) && pair.right == parseSocket(rightName))
                       || (pair.left == parseSocket(rightName) && pair.right == parseSocket(leftName));
            });
            REQUIRE(expected != pairs.end());

            const auto& leftExtrinsics = cameras.at(leftName).value("extrinsics", nlohmann::json::object());
            if(leftExtrinsics.value("to_cam", std::string()) == rightName) {
                REQUIRE(expected->left == parseSocket(leftName));
                REQUIRE(expected->right == parseSocket(rightName));
            }
        }
        ++testedBoards;
    }
    REQUIRE(testedBoards > 0);
}
