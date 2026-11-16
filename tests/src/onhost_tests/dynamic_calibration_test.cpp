#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <depthai/depthai.hpp>
#include <memory>

#include "../../../src/pipeline/node/DynamicCalibrationTransforms.hpp"

#if defined(DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT) && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include "../../../src/pipeline/node/DynamicCalibrationUtils.hpp"

TEST_CASE("DynamicCalibration packs strided image views", "[DynamicCalibrationImages]") {
    const auto channels = GENERATE(1, 3);
    cv::Mat storage(4, 5, CV_MAKETYPE(CV_8U, channels), cv::Scalar(99, 98, 97));
    auto view = storage(cv::Rect(1, 1, 2, 2));
    for(int index = 0; index < 4; ++index) {
        if(channels == 1)
            view.at<std::uint8_t>(index / 2, index % 2) = static_cast<std::uint8_t>(index + 1);
        else
            view.at<cv::Vec3b>(index / 2, index % 2) =
                cv::Vec3b(static_cast<std::uint8_t>(index + 1), static_cast<std::uint8_t>(index + 11), static_cast<std::uint8_t>(index + 21));
    }
    REQUIRE_FALSE(view.isContinuous());

    const auto image = dai::node::DclUtils::cvMatToImageData(view);

    const std::vector<std::uint8_t> expected =
        channels == 1 ? std::vector<std::uint8_t>{1, 2, 3, 4} : std::vector<std::uint8_t>{1, 11, 21, 2, 12, 22, 3, 13, 23, 4, 14, 24};
    REQUIRE(image.data == expected);
    REQUIRE(image.width == 2);
    REQUIRE(image.height == 2);
    REQUIRE(image.format == (channels == 1 ? dcl::DCL_8UC1 : dcl::DCL_8UC3));
}

TEST_CASE("DynamicCalibration preserves metric ToF depth and robust amplitude", "[DynamicCalibrationImages]") {
    cv::Mat amplitude(10, 10, CV_16UC1, cv::Scalar(100));
    amplitude.at<std::uint16_t>(9, 9) = 60000;
    amplitude.at<std::uint16_t>(0, 0) = 50;
    const cv::Mat depth(10, 10, CV_16UC1, cv::Scalar(1234));
    const cv::Mat camera(10, 10, CV_8UC1, cv::Scalar(37));
    const auto observation = dai::node::DclUtils::makeTofObservation(amplitude, depth, camera);
    REQUIRE(observation.depthMeters.size() == 100);
    REQUIRE(observation.depthMeters[0] == Catch::Approx(1.234));
    REQUIRE(observation.amplitude.data[0] == Catch::Approx(128).margin(1));
    REQUIRE(observation.camera.data[0] == 37);
    REQUIRE(depth.at<std::uint16_t>(0, 0) == 1234);
    SECTION("Empty observations are rejected") {
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation({}, {}, {}), std::invalid_argument);
    }
    SECTION("Depth cannot be interpreted as 8-bit image data") {
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(amplitude, camera, camera), std::invalid_argument);
    }
}

TEST_CASE("DynamicCalibration packs strided ToF observations", "[DynamicCalibrationImages]") {
    const auto amplitudeType = GENERATE(CV_8UC1, CV_16UC1);
    cv::Mat amplitudeStorage(4, 6, amplitudeType, cv::Scalar(9));
    cv::Mat depthStorage(4, 6, CV_16UC1, cv::Scalar(9999));
    cv::Mat cameraStorage(4, 5, CV_8UC3, cv::Scalar(99, 98, 97));
    auto amplitude = amplitudeStorage(cv::Rect(1, 1, 3, 2));
    auto depth = depthStorage(cv::Rect(1, 1, 3, 2));
    auto camera = cameraStorage(cv::Rect(1, 1, 2, 2));
    for(int index = 0; index < 6; ++index) {
        if(amplitudeType == CV_8UC1)
            amplitude.at<std::uint8_t>(index / 3, index % 3) = static_cast<std::uint8_t>(10 * (index + 1));
        else
            amplitude.at<std::uint16_t>(index / 3, index % 3) = static_cast<std::uint16_t>(100 * index);
        depth.at<std::uint16_t>(index / 3, index % 3) = static_cast<std::uint16_t>(1000 * index);
    }
    camera.at<cv::Vec3b>(0, 0) = {1, 2, 3};
    camera.at<cv::Vec3b>(0, 1) = {4, 5, 6};
    camera.at<cv::Vec3b>(1, 0) = {7, 8, 9};
    camera.at<cv::Vec3b>(1, 1) = {10, 11, 12};
    REQUIRE_FALSE(amplitude.isContinuous());
    REQUIRE_FALSE(depth.isContinuous());
    REQUIRE_FALSE(camera.isContinuous());

    const auto observation = dai::node::DclUtils::makeTofObservation(amplitude, depth, camera);

    const std::vector<std::uint8_t> expectedAmplitude =
        amplitudeType == CV_8UC1 ? std::vector<std::uint8_t>{10, 20, 30, 40, 50, 60} : std::vector<std::uint8_t>{0, 64, 128, 191, 255, 255};
    REQUIRE(observation.amplitude.data == expectedAmplitude);
    REQUIRE(observation.amplitude.width == 3);
    REQUIRE(observation.amplitude.height == 2);
    REQUIRE(observation.camera.format == dcl::DCL_8UC3);
    REQUIRE(observation.camera.width == 2);
    REQUIRE(observation.camera.height == 2);
    REQUIRE(observation.camera.data == std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
    REQUIRE(observation.depthMeters.size() == 6);
    for(std::size_t index = 0; index < 6; ++index) REQUIRE(observation.depthMeters[index] == Catch::Approx(static_cast<float>(index)));
    REQUIRE(depthStorage.at<std::uint16_t>(0, 0) == 9999);
    REQUIRE(cameraStorage.at<cv::Vec3b>(0, 0) == cv::Vec3b(99, 98, 97));
}

TEST_CASE("DynamicCalibration rejects incompatible ToF observations", "[DynamicCalibrationImages]") {
    const cv::Mat amplitude(2, 3, CV_8UC1, cv::Scalar(37));
    const cv::Mat depth(2, 3, CV_16UC1, cv::Scalar(1234));
    const cv::Mat camera(4, 5, CV_8UC3, cv::Scalar(17, 83, 249));
    SECTION("Unsupported amplitude types") {
        const auto type = GENERATE(CV_8UC3, CV_16SC1, CV_32FC1);
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(cv::Mat(2, 3, type), depth, camera), std::invalid_argument);
    }
    SECTION("Unsupported depth types") {
        const auto type = GENERATE(CV_8UC1, CV_16UC3, CV_32FC1);
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(amplitude, cv::Mat(2, 3, type), camera), std::invalid_argument);
    }
    SECTION("Mismatched depth resolution") {
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(amplitude, cv::Mat(3, 2, CV_16UC1), camera), std::invalid_argument);
    }
    SECTION("Each input is required") {
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation({}, depth, camera), std::invalid_argument);
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(amplitude, {}, camera), std::invalid_argument);
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(amplitude, depth, {}), std::invalid_argument);
    }
    SECTION("Unsupported camera types") {
        const auto type = GENERATE(CV_16UC1, CV_8UC4);
        REQUIRE_THROWS_AS(dai::node::DclUtils::makeTofObservation(amplitude, depth, cv::Mat(4, 5, type)), std::runtime_error);
    }
}

TEST_CASE("DynamicCalibration preserves 8-bit image data", "[DynamicCalibrationImages]") {
    const auto channels = GENERATE(1, 3);
    const cv::Mat input(2, 3, CV_MAKETYPE(CV_8U, channels), cv::Scalar(17, 83, 249));
    const auto image = dai::node::DclUtils::cvMatToImageData(input);
    REQUIRE(image.format == (channels == 1 ? dcl::DCL_8UC1 : dcl::DCL_8UC3));
    REQUIRE(image.data == std::vector<std::uint8_t>(input.data, input.data + input.total() * input.elemSize()));
}
#endif

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
    REQUIRE(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4) == expected);
    REQUIRE(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC2) == Socket::CAM_B);

    using Align = dai::StereoDepthConfig::AlgorithmControl::DepthAlign;
    for(auto alignment : {Align::LEFT, Align::RECTIFIED_LEFT}) {
        REQUIRE(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4, alignment) == Socket::CAM_B);
    }
    for(auto alignment : {Align::RIGHT, Align::RECTIFIED_RIGHT}) {
        REQUIRE(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4, alignment) == Socket::CAM_C);
    }
    REQUIRE_THROWS(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), calibration, dai::Platform::RVC4, Align::CENTER));
    REQUIRE(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), {}, dai::Platform::RVC4) == Socket::CAM_B);

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
        REQUIRE(dai::node::detail::stereoDepthReferenceCamera(stereoPairs.front(), current, dai::Platform::RVC4) == anchor);
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

#if defined(DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT) && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
TEST_CASE("Metric ToF assembly moves only the ToF camera", "[DynamicCalibrationTransforms][MetricTof]") {
    const auto tofSocket = GENERATE(Socket::CAM_A, Socket::CAM_D);
    const auto housingOrigin = GENERATE(Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D);
    const auto housingBase = GENERATE(false, true);
    const auto chainIndex = GENERATE(0, 1, 2);
    const auto platform = GENERATE(dai::Platform::RVC2, dai::Platform::RVC4);
    const std::vector<std::vector<Socket>> chains = {{Socket::CAM_A, Socket::CAM_B, Socket::CAM_C, Socket::CAM_D},
                                                     {Socket::CAM_B, Socket::CAM_C, Socket::CAM_D, Socket::CAM_A},
                                                     {Socket::CAM_D, Socket::CAM_C, Socket::CAM_A, Socket::CAM_B}};
    const auto& chain = chains.at(chainIndex);
    CAPTURE(tofSocket, housingOrigin, housingBase, chainIndex, platform);
    auto eeprom = factoryRig(housingOrigin).getEepromData();
    eeprom.cameraData.clear();
    eeprom.stereoUseSpecTranslation = false;
    eeprom.stereoEnableDistortionCorrection = true;
    eeprom.verticalCameraSocket = Socket::CAM_C;
    eeprom.miscellaneousData = {7, 9};
    dai::CalibrationHandler factory(eeprom);
    for(auto socket : chain) factory.setCameraIntrinsics(socket, {{800, 0, 320}, {0, 810, 240}, {0, 0, 1}}, 640, 480);
    const std::map<Socket, Transform> poses = {{Socket::CAM_A, pose(.10f, .01f, -.02f, .03f)},
                                               {Socket::CAM_B, pose(-.05f, .02f, .01f, -.01f)},
                                               {Socket::CAM_C, pose(.02f, .09f, .01f, -.02f)},
                                               {Socket::CAM_D, pose(-.03f, .03f, -.03f, .02f)}};
    // Rebuild the EEPROM chain; physical relative poses do not depend on its order.
    for(std::size_t index = 0; index < chain.size(); ++index) {
        const auto socket = chain[index];
        const auto next = index + 1 < chain.size() ? chain[index + 1] : Socket::AUTO;
        const auto edge = next == Socket::AUTO ? pose(0, 0, 0, 0) : dai::matrix::matMul(poses.at(next), inverse(poses.at(socket)));
        auto translation = dai::matrix::extractTranslationVector(edge);
        for(auto& value : translation) value *= 100;
        factory.setCameraExtrinsics(socket, next, dai::matrix::extractRotationMatrix(edge), translation, {21, 32, 43});
        factory.setDistortionCoefficients(socket, {.01f, -.02f, .001f, -.001f, .005f});
    }
    factory.setStereoLeft(Socket::CAM_B, dai::matrix::extractRotationMatrix(pose(.03f, 0, 0, 0)));
    factory.setStereoRight(Socket::CAM_C, dai::matrix::extractRotationMatrix(pose(-.02f, 0, 0, 0)));
    auto current = factory;
    // Preserve active user calibration rather than restoring factory links.
    current.updateCameraExtrinsics(chain.front(), chain[1], dai::matrix::extractRotationMatrix(pose(.08f, 0, 0, 0)), {8, 2, -1});
    const auto before = current.getEepromData();
    const auto originalJson = current.eepromToJson();
    const auto base = housingBase ? std::variant<Socket, dai::HousingCoordinateSystem>(dai::HousingCoordinateSystem::AUTO)
                                  : std::variant<Socket, dai::HousingCoordinateSystem>(Socket::CAM_C);
    const auto fixedPose = dai::node::DclUtils::computeBaseToSocketTransform(current, base, Socket::CAM_C);
    const auto candidate = dai::matrix::matMul(pose(.006f, .002f, -.001f, .003f), dai::node::DclUtils::computeBaseToSocketTransform(current, base, tofSocket));
    std::map<Socket, Transform> calibrated = {{tofSocket, candidate}, {Socket::CAM_C, fixedPose}};
    // Model the metric node path: complete unobserved poses from CURRENT calibration.
    for(auto socket : chain)
        if(!calibrated.count(socket)) calibrated.emplace(socket, dai::node::DclUtils::computeBaseToSocketTransform(current, base, socket));

    const auto assembled = dai::node::detail::assembleDynamicCalibration(current, {}, chain, calibrated, housingBase, {}, platform);
    const auto result = dai::CalibrationHandler::fromJson(assembled.eepromToJson());

    for(auto source : chain) {
        for(auto destination : chain) {
            if(source == destination) continue;
            if(source != tofSocket && destination != tofSocket)
                requireTransform(result.getCameraExtrinsics(source, destination, false, dai::LengthUnit::METER),
                                 current.getCameraExtrinsics(source, destination, false, dai::LengthUnit::METER));
            else
                requireTransform(result.getCameraExtrinsics(source, destination, false, dai::LengthUnit::METER),
                                 dai::matrix::matMul(calibrated.at(destination), inverse(calibrated.at(source))));
        }
        if(housingBase)
            requireTransform(result.getHousingCalibration(source, dai::HousingCoordinateSystem::AUTO, false, dai::LengthUnit::METER),
                             inverse(calibrated.at(source)));
    }
    REQUIRE(result.getCameraExtrinsics(tofSocket, Socket::CAM_C, false) != current.getCameraExtrinsics(tofSocket, Socket::CAM_C, false));
    // Mask only estimated transforms: all intrinsics, distortion, design translations,
    // stereo fields, chain sockets, IMU data and other EEPROM metadata must remain exact.
    auto metadata = result.getEepromData();
    for(auto socket : chain) {
        metadata.cameraData.at(socket).extrinsics.rotationMatrix = before.cameraData.at(socket).extrinsics.rotationMatrix;
        metadata.cameraData.at(socket).extrinsics.translation = before.cameraData.at(socket).extrinsics.translation;
    }
    if(housingBase) {
        metadata.housingExtrinsics.rotationMatrix = before.housingExtrinsics.rotationMatrix;
        metadata.housingExtrinsics.translation = before.housingExtrinsics.translation;
    }
    const auto canonicalMetadata = [](nlohmann::json json) {
        // JSON encodes the unordered socket map as an array; reloads can reorder it.
        auto& cameras = json["cameraData"];
        std::sort(cameras.begin(), cameras.end(), [](const auto& a, const auto& b) { return a.at(0) < b.at(0); });
        return json;
    };
    REQUIRE(canonicalMetadata(dai::CalibrationHandler(metadata).eepromToJson()) == canonicalMetadata(originalJson));
    REQUIRE(current.eepromToJson() == originalJson);
}
#endif

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
