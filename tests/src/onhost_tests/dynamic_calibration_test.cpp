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

void requireTransform(const Transform& actual, const Transform& expected, float translationTolerance = 1e-5f) {
    REQUIRE(actual.size() == expected.size());
    for(size_t row = 0; row < expected.size(); ++row) {
        REQUIRE(actual[row].size() == expected[row].size());
        for(size_t col = 0; col < expected[row].size(); ++col) {
            REQUIRE(actual[row][col] == Catch::Approx(expected[row][col]).margin(col == 3 && row < 3 ? translationTolerance : 1e-5f));
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
    const auto result = dai::node::detail::assembleDynamicCalibration(current, factory, sockets, dclPoses, true, stereoPairs, dai::Platform::RVC2);
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
    const auto repeated = dai::node::detail::assembleDynamicCalibration(result, factory, sockets, dclPoses, true, stereoPairs, dai::Platform::RVC2);
    requireTransform(repeated.getHousingCalibration(Socket::CAM_A, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER),
                     result.getHousingCalibration(Socket::CAM_A, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER));
}

TEST_CASE("DynamicCalibration factory link requirements", "[DynamicCalibrationTransforms]") {
    const auto current = factoryRig(Socket::CAM_A);
    const std::vector<Socket> sockets = {Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D};
    std::map<Socket, Transform> poses;
    for(size_t i = 0; i < sockets.size(); ++i) poses[sockets[i]] = pose(0.1f * i, 0.02f * i, 0, 0);
    SECTION("Fully observed rig needs no factory calibration") {
        const auto result = dai::node::detail::assembleDynamicCalibration(current, {}, sockets, poses, true, {}, dai::Platform::RVC2);
        for(const auto& entry : poses) {
            requireTransform(result.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER),
                             inverse(entry.second));
        }
    }
    SECTION("Unobserved cameras require a stereo pair") {
        poses.erase(Socket::CAM_A);
        REQUIRE_THROWS_WITH(dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, true, {}, dai::Platform::RVC2),
                            "DynamicCalibration requires a stereo pair to anchor unobserved cameras to factory extrinsics.");
    }
    SECTION("Missing factory links fail explicitly") {
        poses.erase(Socket::CAM_A);
        REQUIRE_THROWS(dai::node::detail::assembleDynamicCalibration(current, {}, sockets, poses, true, stereoPairs, dai::Platform::RVC2));
    }
    SECTION("No housing preserves relative poses") {
        poses.erase(Socket::CAM_A);
        const auto result = dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, false, stereoPairs, dai::Platform::RVC2);
        requireTransform(result.getCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, false, dai::LengthUnit::METER),
                         dai::matrix::matMul(poses.at(Socket::CAM_C), inverse(poses.at(Socket::CAM_B))));
    }
    SECTION("Unobserved cameras require a stereo pair whose reference camera is calibrated") {
        poses.erase(Socket::CAM_B);
        REQUIRE_THROWS_WITH(dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, true, stereoPairs, dai::Platform::RVC2),
                            "DynamicCalibration requires a stereo pair whose default depth reference camera is a calibration input to anchor unobserved "
                            "cameras to factory extrinsics.");
    }
    SECTION("Anchor falls back to a later stereo pair whose reference camera is calibrated") {
        // The device's first pair (B, C) references CAM_B, which is not a calibration input; (A, D) references CAM_A, which is.
        poses.erase(Socket::CAM_B);
        const std::vector<dai::StereoPair> devicePairs = {{Socket::CAM_B, Socket::CAM_C}, {Socket::CAM_A, Socket::CAM_D}};
        const auto result = dai::node::detail::assembleDynamicCalibration(current, current, sockets, poses, true, devicePairs, dai::Platform::RVC2);
        // CAM_B follows the CAM_A anchor through its factory transform, while the calibrated links keep the DCL poses.
        requireTransform(result.getCameraExtrinsics(Socket::CAM_A, Socket::CAM_B, false, dai::LengthUnit::METER),
                         current.getCameraExtrinsics(Socket::CAM_A, Socket::CAM_B, false, dai::LengthUnit::METER));
        requireTransform(result.getCameraExtrinsics(Socket::CAM_C, Socket::CAM_D, false, dai::LengthUnit::METER),
                         dai::matrix::matMul(poses.at(Socket::CAM_D), inverse(poses.at(Socket::CAM_C))));
    }
}

TEST_CASE("DynamicCalibration anchors ToF to B in the B-C-D-A device chain", "[DynamicCalibrationTransforms]") {
    auto factory = factoryRig(Socket::CAM_A);
    const auto rotation = dai::matrix::extractRotationMatrix(pose(0.2f, 0, 0, 0));
    factory.setCameraExtrinsics(Socket::CAM_A, Socket::AUTO, rotation, {0, 0, 0});
    factory.setCameraExtrinsics(Socket::CAM_D, Socket::CAM_A, rotation, {2, 1, 3});
    const std::vector<Socket> sockets = {Socket::CAM_B, Socket::CAM_C, Socket::CAM_D, Socket::CAM_A};
    const std::map<Socket, Transform> poses = {{Socket::CAM_B, pose(0.4f, 0.02f, 0.03f, -0.01f)}, {Socket::CAM_C, pose(-0.3f, 0.09f, -0.02f, 0.04f)}};
    const auto result = dai::node::detail::assembleDynamicCalibration(factory, factory, sockets, poses, true, stereoPairs, dai::Platform::RVC2);
    for(auto socket : {Socket::CAM_D, Socket::CAM_A}) {
        requireTransform(result.getCameraExtrinsics(Socket::CAM_B, socket), factory.getCameraExtrinsics(Socket::CAM_B, socket));
    }
    for(const auto& entry : poses) {
        requireTransform(result.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER), inverse(entry.second));
    }
    requireTransform(result.getCameraExtrinsics(Socket::CAM_D, Socket::CAM_A), factory.getCameraExtrinsics(Socket::CAM_D, Socket::CAM_A));
}

TEST_CASE("Stereo depth preserves requested alignment on the wire", "[DynamicCalibrationTransforms]") {
    using Align = dai::StereoDepthConfig::AlgorithmControl::DepthAlign;
    const auto alignment = GENERATE(Align::AUTO, Align::LEFT, Align::RIGHT, Align::RECTIFIED_LEFT, Align::RECTIFIED_RIGHT, Align::CENTER);
    dai::StereoDepthConfig config;
    config.algorithmControl.depthAlign = alignment;
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype;
    config.serialize(metadata, datatype);
    dai::StereoDepthConfig decoded;
    dai::utility::deserialize(metadata, decoded);
    REQUIRE(decoded.algorithmControl.depthAlign == alignment);
    REQUIRE(config.algorithmControl.depthAlign == alignment);
    REQUIRE(datatype == dai::DatatypeEnum::StereoDepthConfig);
    struct StereoDepthAccess : dai::node::StereoDepth {
        StereoDepthAccess() : dai::node::StereoDepth(std::make_unique<dai::StereoDepthProperties>()) {}
        using dai::node::StereoDepth::getProperties;
    };
    StereoDepthAccess stereo;
    stereo.initialConfig->algorithmControl.depthAlign = alignment;
    REQUIRE(stereo.getProperties().initialConfig.algorithmControl.depthAlign == alignment);
    REQUIRE(stereo.initialConfig->algorithmControl.depthAlign == alignment);
}

TEST_CASE("Stereo AUTO reference follows platform and measured CAM_A proximity", "[DynamicCalibrationTransforms]") {
    // Left is 10 cm from CAM_A. These right distances cover both sides of
    // the strict 80% boundary, including equal camera distances.
    const auto rightDistance = GENERATE(7.0f, 8.0f, 9.0f, 10.0f, 12.0f);
    auto calibration = factoryRig(Socket::CAM_A);
    const auto identity = dai::matrix::extractRotationMatrix(pose(0, 0, 0, 0));
    calibration.updateCameraExtrinsics(Socket::CAM_A, Socket::CAM_B, identity, {10, 0, 0});
    calibration.updateCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, identity, {rightDistance - 10, 0, 0});
    const auto expected = rightDistance == 7.0f ? Socket::CAM_C : Socket::CAM_B;
    REQUIRE(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4) == expected);
    REQUIRE(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC2) == Socket::CAM_B);

    using Align = dai::StereoDepthConfig::AlgorithmControl::DepthAlign;
    for(auto alignment : {Align::LEFT, Align::RECTIFIED_LEFT}) {
        REQUIRE(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4, alignment) == Socket::CAM_B);
    }
    for(auto alignment : {Align::RIGHT, Align::RECTIFIED_RIGHT}) {
        REQUIRE(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4, alignment) == Socket::CAM_C);
    }
    REQUIRE_THROWS(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4, Align::CENTER));
    REQUIRE(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), {}, dai::Platform::RVC4) == Socket::CAM_B);

    // Selection uses active calibration, while passive-camera links use factory data.
    const auto factory = factoryRig(Socket::CAM_A);
    const std::vector<Socket> sockets = {Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D};
    const std::map<Socket, Transform> poses = {{Socket::CAM_B, pose(0.4f, 0.02f, 0.03f, -0.01f)}, {Socket::CAM_C, pose(-0.3f, 0.09f, -0.02f, 0.04f)}};
    const auto result = dai::node::detail::assembleDynamicCalibration(calibration, factory, sockets, poses, true, stereoPairs, dai::Platform::RVC4);
    for(auto socket : {Socket::CAM_A, Socket::CAM_D}) {
        requireTransform(result.getCameraExtrinsics(expected, socket), factory.getCameraExtrinsics(expected, socket));
    }
    for(const auto& entry : poses) {
        requireTransform(result.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER), inverse(entry.second));
    }
}

TEST_CASE("Simulated DCL updates preserve factory calibration queries across reloads", "[DynamicCalibrationTransforms][SimulatedDcl]") {
    const auto anchor = GENERATE(Socket::CAM_B, Socket::CAM_C);
    const auto housingOrigin = GENERATE(Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D);
    const auto chainIndex = GENERATE(0, 1, 2);
    const std::vector<std::vector<Socket>> chains = {{Socket::CAM_B, Socket::CAM_C, Socket::CAM_D, Socket::CAM_A},
                                                     {Socket::CAM_A, Socket::CAM_D, Socket::CAM_C, Socket::CAM_B},
                                                     {Socket::CAM_C, Socket::CAM_A, Socket::CAM_B, Socket::CAM_D}};
    const auto& chain = chains.at(chainIndex);
    CAPTURE(anchor, housingOrigin, chainIndex);

    // Define the physical rig in housing coordinates, independently of the EEPROM chain.
    // CAM_A is close enough to the requested anchor to unambiguously select it on RVC4.
    const std::map<Socket, Transform> factoryPoses = {{Socket::CAM_A, pose(0.12f, anchor == Socket::CAM_B ? 0.01f : 0.11f, 0.003f, -0.002f)},
                                                      {Socket::CAM_B, pose(-0.08f, 0, 0, 0)},
                                                      {Socket::CAM_C, pose(0.06f, 0.10f, 0.001f, 0.002f)},
                                                      {Socket::CAM_D, pose(-0.15f, 0.04f, -0.025f, 0.018f)}};
    dai::EepromData eeprom;
    eeprom.housingExtrinsics.toCameraSocket = housingOrigin;
    const auto& housingPose = factoryPoses.at(housingOrigin);
    eeprom.housingExtrinsics.rotationMatrix = dai::matrix::extractRotationMatrix(housingPose);
    eeprom.housingExtrinsics.translation = {100 * housingPose[0][3], 100 * housingPose[1][3], 100 * housingPose[2][3]};
    dai::CalibrationHandler factory(eeprom);
    for(auto socket : chain) factory.setCameraIntrinsics(socket, {{800, 0, 320}, {0, 810, 240}, {0, 0, 1}}, 640, 480);
    for(size_t i = 0; i + 1 < chain.size(); ++i) {
        const auto edge = dai::matrix::matMul(factoryPoses.at(chain[i + 1]), inverse(factoryPoses.at(chain[i])));
        auto translation = dai::matrix::extractTranslationVector(edge);
        for(auto& value : translation) value *= 100;
        // Deliberately different design translations catch accidental useSpecTranslation=true.
        factory.setCameraExtrinsics(chain[i], chain[i + 1], dai::matrix::extractRotationMatrix(edge), translation, {20, 30, 40});
    }
    const auto factoryJson = factory.eepromToJson();
    auto current = dai::CalibrationHandler::fromJson(factoryJson);
    // Simulate a previously modified user calibration, so copying its links cannot pass.
    for(size_t i = 0; i + 1 < chain.size(); ++i) {
        auto edge = current.getCameraExtrinsics(chain[i], chain[i + 1], false);
        auto translation = dai::matrix::extractTranslationVector(edge);
        translation[1] += 0.1f;
        current.updateCameraExtrinsics(chain[i], chain[i + 1], dai::matrix::extractRotationMatrix(edge), translation);
    }
    for(auto socket : {Socket::CAM_A, Socket::CAM_D}) {
        REQUIRE(current.getCameraExtrinsics(anchor, socket, false) != factory.getCameraExtrinsics(anchor, socket, false));
    }

    for(int cycle = 1; cycle <= 3; ++cycle) {
        CAPTURE(cycle);
        REQUIRE(dai::utility::stereoDepthReferenceCamera(stereoPairs.front(), current, dai::Platform::RVC4) == anchor);
        // Mock DCL output with a new stereo transform and a changing housing pose.
        // The expected B->C transform is specified directly, not derived from the result.
        const auto expectedStereo = pose(0.02f * cycle, 0.10f + 0.001f * cycle, -0.002f, 0.003f);
        const auto housingToB = pose(-0.04f * cycle, 0.002f * cycle, -0.003f, 0.004f);
        const std::map<Socket, Transform> dclOutput = {{Socket::CAM_B, housingToB}, {Socket::CAM_C, dai::matrix::matMul(expectedStereo, housingToB)}};
        REQUIRE(current.getCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, false, dai::LengthUnit::METER) != expectedStereo);
        const auto updated = dai::node::detail::assembleDynamicCalibration(current, factory, chain, dclOutput, true, stereoPairs, dai::Platform::RVC4);
        // Query a freshly loaded CalibrationHandler, as downstream calibration consumers do.
        current = dai::CalibrationHandler::fromJson(updated.eepromToJson());
        requireTransform(current.getCameraExtrinsics(Socket::CAM_B, Socket::CAM_C, false, dai::LengthUnit::METER), expectedStereo);
        requireTransform(current.getCameraExtrinsics(Socket::CAM_C, Socket::CAM_B, false, dai::LengthUnit::METER), inverse(expectedStereo));
        for(auto socket : {Socket::CAM_A, Socket::CAM_D}) {
            CAPTURE(socket);
            for(auto unit : {dai::LengthUnit::METER, dai::LengthUnit::CENTIMETER}) {
                // Allow 1 micrometer of translation roundoff in centimeter queries;
                // rotation tolerance stays unchanged regardless of translation units.
                const auto tolerance = unit == dai::LengthUnit::CENTIMETER ? 1e-4f : 1e-5f;
                requireTransform(current.getCameraExtrinsics(anchor, socket, false, unit), factory.getCameraExtrinsics(anchor, socket, false, unit), tolerance);
                requireTransform(current.getCameraExtrinsics(socket, anchor, false, unit), factory.getCameraExtrinsics(socket, anchor, false, unit), tolerance);
            }
            // The other stereo camera's links must follow the NEW stereo transform.
            const auto other = anchor == Socket::CAM_B ? Socket::CAM_C : Socket::CAM_B;
            const auto otherToAnchor = anchor == Socket::CAM_B ? inverse(expectedStereo) : expectedStereo;
            requireTransform(current.getCameraExtrinsics(other, socket, false, dai::LengthUnit::METER),
                             dai::matrix::matMul(factory.getCameraExtrinsics(anchor, socket, false, dai::LengthUnit::METER), otherToAnchor));
        }
        requireTransform(current.getCameraExtrinsics(Socket::CAM_A, Socket::CAM_D, false), factory.getCameraExtrinsics(Socket::CAM_A, Socket::CAM_D, false));
        for(const auto& entry : dclOutput) {
            requireTransform(current.getHousingCalibration(entry.first, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER),
                             inverse(entry.second));
        }
        for(auto socket : chain) requireTransform(current.getCameraIntrinsics(socket), factory.getCameraIntrinsics(socket));
        REQUIRE(factory.eepromToJson() == factoryJson);
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
