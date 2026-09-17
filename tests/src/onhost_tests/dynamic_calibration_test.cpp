#include <catch2/catch_all.hpp>
#include <cmath>
#include <depthai/depthai.hpp>
#include <memory>

#include "../../../src/pipeline/node/DynamicCalibrationTransforms.hpp"

namespace {
using Socket = dai::CameraBoardSocket;
using Transform = std::vector<std::vector<float>>;
const std::vector<dai::StereoPair> stereoPairs = {{Socket::CAM_B, Socket::CAM_C}};

Transform pose(float angle, float x, float y, float z) {
    const auto c = std::cos(angle);
    const auto s = std::sin(angle);
    const auto cy = std::cos(angle * 0.5f);
    const auto sy = std::sin(angle * 0.5f);
    return {{c * cy, -s, c * sy, x}, {s * cy, c, s * sy, y}, {-sy, 0, cy, z}, {0, 0, 0, 1}};
}

Transform inverse(Transform transform) {
    dai::matrix::invertSe3Matrix4x4InPlace(transform);
    return transform;
}

void requireTransform(const Transform& actual, const Transform& expected) {
    REQUIRE(actual.size() == expected.size());
    for(size_t row = 0; row < expected.size(); ++row) {
        REQUIRE(actual[row].size() == expected[row].size());
        for(size_t col = 0; col < expected[row].size(); ++col) {
            REQUIRE(actual[row][col] == Catch::Approx(expected[row][col]).margin(1e-5));
        }
    }
}

dai::CalibrationHandler factoryRig(Socket housingOrigin) {
    dai::EepromData eeprom;
    eeprom.housingExtrinsics.toCameraSocket = housingOrigin;
    eeprom.housingExtrinsics.rotationMatrix = dai::matrix::extractRotationMatrix(pose(0.1f, 0, 0, 0));
    eeprom.housingExtrinsics.translation = {1, 2, 3};
    dai::CalibrationHandler handler(eeprom);
    const Transform intrinsics = {{800, 0, 320}, {0, 800, 240}, {0, 0, 1}};
    for(auto socket : {Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D}) {
        handler.setCameraIntrinsics(socket, intrinsics, 640, 480);
    }
    handler.setCameraExtrinsics(Socket::CAM_A, Socket::CAM_B, dai::matrix::extractRotationMatrix(pose(0.2f, 0, 0, 0)), {3, 1, 2}, {4, 0, 0});
    handler.setCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, dai::matrix::extractRotationMatrix(pose(-0.1f, 0, 0, 0)), {7, 0, 1});
    handler.setCameraExtrinsics(Socket::CAM_C, Socket::CAM_D, dai::matrix::extractRotationMatrix(pose(0.3f, 0, 0, 0)), {2, -1, 3});
    return handler;
}
}  // namespace

TEST_CASE("DynamicCalibration preserves DCL housing poses and factory links", "[DynamicCalibrationTransforms]") {
    const auto anchor = GENERATE(Socket::CAM_B, Socket::CAM_C);
    const std::vector<dai::StereoPair> stereoPairs = {{anchor, Socket::CAM_A}, {Socket::CAM_A, Socket::CAM_D}};
    const auto housingOrigin = GENERATE(Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D);
    const auto factory = factoryRig(housingOrigin);
    auto current = factory;
    // The user calibration differs from factory: restoring the current link would be wrong.
    current.updateCameraExtrinsics(Socket::CAM_A, Socket::CAM_B, dai::matrix::extractRotationMatrix(pose(-0.4f, 0, 0, 0)), {9, 8, 7});
    const std::vector<Socket> sockets = {Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D};
    const std::map<Socket, Transform> dclPoses = {{Socket::CAM_B, pose(0.4f, 0.02f, 0.03f, -0.01f)}, {Socket::CAM_C, pose(-0.3f, 0.09f, -0.02f, 0.04f)}};
    const auto result = dai::node::detail::assembleDynamicCalibration(current, factory, sockets, dclPoses, true, stereoPairs);
    for(const auto& entry : dclPoses) {
        requireTransform(result.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER), inverse(entry.second));
    }
    for(const auto& pair : {std::make_pair(anchor, Socket::CAM_A), std::make_pair(anchor, Socket::CAM_D)}) {
        requireTransform(result.getCameraExtrinsics(pair.first, pair.second), factory.getCameraExtrinsics(pair.first, pair.second));
    }
    requireTransform(result.getCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, false, dai::LengthUnit::METER),
                     dai::matrix::matMul(dclPoses.at(Socket::CAM_C), inverse(dclPoses.at(Socket::CAM_B))));
    // Existing design translations and intrinsics are not calibration estimates.
    requireTransform(result.getCameraIntrinsics(Socket::CAM_A), current.getCameraIntrinsics(Socket::CAM_A));
    const auto actualSpec = result.getEepromData().cameraData.at(Socket::CAM_A).extrinsics.specTranslation;
    const auto originalSpec = current.getEepromData().cameraData.at(Socket::CAM_A).extrinsics.specTranslation;
    REQUIRE(actualSpec.x == originalSpec.x);
    REQUIRE(actualSpec.y == originalSpec.y);
    REQUIRE(actualSpec.z == originalSpec.z);
    const auto repeated = dai::node::detail::assembleDynamicCalibration(result, factory, sockets, dclPoses, true, stereoPairs);
    requireTransform(repeated.getHousingCalibration(Socket::CAM_A, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER),
                     result.getHousingCalibration(Socket::CAM_A, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER));
}

TEST_CASE("DynamicCalibration factory link requirements", "[DynamicCalibrationTransforms]") {
    const auto current = factoryRig(Socket::CAM_A);
    const std::vector<Socket> sockets = {Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D};
    std::map<Socket, Transform> poses;
    for(size_t i = 0; i < sockets.size(); ++i) poses[sockets[i]] = pose(0.1f * i, 0.02f * i, 0, 0);
    SECTION("Fully observed rig needs no factory calibration") {
        const auto result = dai::node::detail::assembleDynamicCalibration(current, {}, sockets, poses, true, {});
        for(const auto& entry : poses) {
            requireTransform(result.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER),
                             inverse(entry.second));
        }
    }
    SECTION("Unobserved cameras require a stereo pair") {
        poses.erase(Socket::CAM_A);
        REQUIRE_THROWS_WITH(dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, true, {}),
                            "DynamicCalibration requires a stereo pair to anchor unobserved cameras to factory extrinsics.");
    }
    SECTION("Missing factory links fail explicitly") {
        poses.erase(Socket::CAM_A);
        REQUIRE_THROWS(dai::node::detail::assembleDynamicCalibration(current, {}, sockets, poses, true, stereoPairs));
    }
    SECTION("No housing preserves relative poses") {
        poses.erase(Socket::CAM_A);
        const auto result = dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, false, stereoPairs);
        requireTransform(result.getCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, false, dai::LengthUnit::METER),
                         dai::matrix::matMul(poses.at(Socket::CAM_C), inverse(poses.at(Socket::CAM_B))));
    }
    SECTION("Unobserved cameras require the first stereo pair anchor to be calibrated") {
        poses.erase(Socket::CAM_B);
        REQUIRE_THROWS_WITH(dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, true, stereoPairs),
                            "DynamicCalibration requires the first stereo pair's default depth reference camera as a calibration input to anchor unobserved "
                            "cameras to factory extrinsics.");
    }
}

TEST_CASE("DynamicCalibration anchors ToF to B in the B-C-D-A device chain", "[DynamicCalibrationTransforms]") {
    auto factory = factoryRig(Socket::CAM_A);
    const auto rotation = dai::matrix::extractRotationMatrix(pose(0.2f, 0, 0, 0));
    factory.setCameraExtrinsics(Socket::CAM_A, Socket::AUTO, rotation, {0, 0, 0});
    factory.setCameraExtrinsics(Socket::CAM_D, Socket::CAM_A, rotation, {2, 1, 3});
    const std::vector<Socket> sockets = {Socket::CAM_B, Socket::CAM_C, Socket::CAM_D, Socket::CAM_A};
    const std::map<Socket, Transform> poses = {{Socket::CAM_B, pose(0.4f, 0.02f, 0.03f, -0.01f)}, {Socket::CAM_C, pose(-0.3f, 0.09f, -0.02f, 0.04f)}};
    const auto result = dai::node::detail::assembleDynamicCalibration(factory, factory, sockets, poses, true, stereoPairs);
    for(auto socket : {Socket::CAM_D, Socket::CAM_A}) {
        requireTransform(result.getCameraExtrinsics(Socket::CAM_B, socket), factory.getCameraExtrinsics(Socket::CAM_B, socket));
    }
    for(const auto& entry : poses) {
        requireTransform(result.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER), inverse(entry.second));
    }
    requireTransform(result.getCameraExtrinsics(Socket::CAM_D, Socket::CAM_A), factory.getCameraExtrinsics(Socket::CAM_D, Socket::CAM_A));
}

TEST_CASE("Stereo depth wire configuration and calibration share the default reference", "[DynamicCalibrationTransforms]") {
    using Align = dai::StereoDepthConfig::AlgorithmControl::DepthAlign;
    const auto alignment = GENERATE(Align::AUTO, Align::LEFT, Align::RIGHT, Align::RECTIFIED_LEFT, Align::RECTIFIED_RIGHT, Align::CENTER);
    dai::StereoDepthConfig config;
    config.algorithmControl.depthAlign = alignment;
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype;
    config.serialize(metadata, datatype);
    dai::StereoDepthConfig decoded;
    dai::utility::deserialize(metadata, decoded);
    const auto resolved = dai::utility::resolveStereoDepthAlignment(alignment);
    REQUIRE(decoded.algorithmControl.depthAlign == resolved);
    REQUIRE(config.algorithmControl.depthAlign == alignment);
    REQUIRE(datatype == dai::DatatypeEnum::StereoDepthConfig);
    struct StereoDepthAccess : dai::node::StereoDepth {
        StereoDepthAccess() : dai::node::StereoDepth(std::make_unique<dai::StereoDepthProperties>()) {}
        using dai::node::StereoDepth::getProperties;
    };
    StereoDepthAccess stereo;
    stereo.initialConfig->algorithmControl.depthAlign = alignment;
    REQUIRE(stereo.getProperties().initialConfig.algorithmControl.depthAlign == resolved);
    REQUIRE(stereo.initialConfig->algorithmControl.depthAlign == alignment);
    const dai::StereoPair pair{Socket::CAM_C, Socket::CAM_D};
    if(resolved == Align::CENTER) {
        REQUIRE_THROWS(dai::utility::stereoDepthReferenceCamera(pair, alignment));
    } else {
        const auto expected = (resolved == Align::LEFT || resolved == Align::RECTIFIED_LEFT) ? pair.left : pair.right;
        REQUIRE(dai::utility::stereoDepthReferenceCamera(pair, alignment) == expected);
    }
}

TEST_CASE("DynamicCalibration - Commands", "[DynamicCalibrationControl]") {
    using DCC = dai::DynamicCalibrationControl;

    SECTION("Calibrate command") {
        auto defaultCmd = DCC::calibrate();
        REQUIRE(std::holds_alternative<DCC::Commands::Calibrate>(defaultCmd->command));
        auto& defaultCalibrate = std::get<DCC::Commands::Calibrate>(defaultCmd->command);
        REQUIRE(defaultCalibrate.keepCameraCenters == true);

        auto cmd = DCC::calibrate(true, false);
        REQUIRE(std::holds_alternative<DCC::Commands::Calibrate>(cmd->command));
        auto& c = std::get<DCC::Commands::Calibrate>(cmd->command);
        REQUIRE(c.force == true);
        REQUIRE(c.keepCameraCenters == false);
    }

    SECTION("CalibrationQuality command") {
        auto cmd = DCC::calibrationQuality(false);
        REQUIRE(std::holds_alternative<DCC::Commands::CalibrationQuality>(cmd->command));
        auto& c = std::get<DCC::Commands::CalibrationQuality>(cmd->command);
        REQUIRE(c.force == false);
    }

    SECTION("StartCalibration command with custom periods") {
        auto cmd = DCC::startCalibration(1.0f, 10.0f);
        REQUIRE(std::holds_alternative<DCC::Commands::StartCalibration>(cmd->command));
        auto& c = std::get<DCC::Commands::StartCalibration>(cmd->command);
        REQUIRE(c.loadImagePeriod == Catch::Approx(1.0f));
        REQUIRE(c.calibrationPeriod == Catch::Approx(10.0f));
        REQUIRE(c.keepCameraCenters == true);

        auto movableCentersCmd = DCC::startCalibration(1.0f, 10.0f, false);
        auto& movableCenters = std::get<DCC::Commands::StartCalibration>(movableCentersCmd->command);
        REQUIRE(movableCenters.keepCameraCenters == false);
    }

    SECTION("StopCalibration command") {
        auto cmd = DCC::stopCalibration();
        REQUIRE(std::holds_alternative<DCC::Commands::StopCalibration>(cmd->command));
    }

    SECTION("LoadImage command") {
        auto cmd = DCC::loadImage();
        REQUIRE(std::holds_alternative<DCC::Commands::LoadImage>(cmd->command));
    }

    SECTION("ResetData command") {
        auto cmd = DCC::resetData();
        REQUIRE(std::holds_alternative<DCC::Commands::ResetData>(cmd->command));
    }

    SECTION("SetPerformanceMode command") {
        auto cmd = DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_SPEED);
        REQUIRE(std::holds_alternative<DCC::Commands::SetPerformanceMode>(cmd->command));
        auto& c = std::get<DCC::Commands::SetPerformanceMode>(cmd->command);
        REQUIRE(c.performanceMode == DCC::PerformanceMode::OPTIMIZE_SPEED);
    }

    SECTION("ApplyCalibration command") {
        dai::CalibrationHandler calHandler;  // Assuming default constructible
        auto cmd = DCC::applyCalibration(calHandler);
        REQUIRE(std::holds_alternative<DCC::Commands::ApplyCalibration>(cmd->command));
        auto& c = std::get<DCC::Commands::ApplyCalibration>(cmd->command);
        // Optionally verify that calibration matches (if operator== is defined)
    }
    SECTION("ComputeMetricCommand command") {
        dai::CalibrationHandler calHandler;  // Assuming default constructible
        auto cmd = DCC::computeCalibrationMetrics(calHandler);
        REQUIRE(std::holds_alternative<DCC::Commands::ComputeCalibrationMetrics>(cmd->command));
        auto& c = std::get<DCC::Commands::ComputeCalibrationMetrics>(cmd->command);
    }
}
