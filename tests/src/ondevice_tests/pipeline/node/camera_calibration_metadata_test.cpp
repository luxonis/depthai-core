#include <algorithm>
#include <catch2/catch_all.hpp>
#include <vector>

#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

TEST_CASE("Camera frames carry the session calibration through a center crop", "[camera-hil]") {
    // A square output crops the sides of every landscape sensor mode
    constexpr unsigned int SIDE = 240;
    constexpr float FOCAL_X = 800.0f;
    constexpr float FOCAL_Y = 810.0f;
    const std::vector<float> distortion{0.01f, -0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const std::vector<float> translation{1.0f, 2.0f, 3.0f};
    const std::vector<float> specTranslation{4.0f, 5.0f, 6.0f};
    const std::vector<std::vector<float>> identity{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    dai::Pipeline p;
    const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());
    const auto& feature = features.front();
    REQUIRE_FALSE(feature.configs.empty());
    const auto& mode = feature.configs.front();
    CAPTURE(feature.socket, mode.width, mode.height);
    const auto modeWidth = static_cast<float>(mode.width);
    const auto modeHeight = static_cast<float>(mode.height);
    const auto origin = feature.socket == dai::CameraBoardSocket::CAM_B ? dai::CameraBoardSocket::CAM_C : dai::CameraBoardSocket::CAM_B;

    dai::CalibrationHandler calibration;
    calibration.setCameraIntrinsics(
        feature.socket, {{FOCAL_X, 0.0f, modeWidth / 2.0f}, {0.0f, FOCAL_Y, modeHeight / 2.0f}, {0.0f, 0.0f, 1.0f}}, mode.width, mode.height);
    calibration.setDistortionCoefficients(feature.socket, distortion);
    calibration.setCameraIntrinsics(origin, {{FOCAL_X, 0.0f, 160.0f}, {0.0f, FOCAL_Y, 120.0f}, {0.0f, 0.0f, 1.0f}}, 320, 240);
    calibration.setCameraExtrinsics(feature.socket, origin, identity, translation, specTranslation);
    p.setCalibrationData(calibration);

    auto camera = p.create<dai::node::Camera>()->build(feature.socket, std::pair(mode.width, mode.height));
    auto* output = camera->requestOutput({SIDE, SIDE}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP);
    REQUIRE(output != nullptr);
    auto queue = output->createOutputQueue(2, false);
    p.start();

    const auto frame = camera_test::nextFrame(queue);
    REQUIRE(frame->getWidth() == SIDE);
    REQUIRE(frame->getHeight() == SIDE);
    const auto& transformation = frame->getTransformation();

    // Cropping keeps the scale of the shorter side and the center of the image
    const float scale = std::max(SIDE / modeWidth, SIDE / modeHeight);
    const auto intrinsics = transformation.getIntrinsicMatrix();
    REQUIRE(intrinsics[0][0] == Catch::Approx(FOCAL_X * scale));
    REQUIRE(intrinsics[1][1] == Catch::Approx(FOCAL_Y * scale));
    REQUIRE(intrinsics[0][2] == Catch::Approx(SIDE / 2.0f).margin(1e-3));
    REQUIRE(intrinsics[1][2] == Catch::Approx(SIDE / 2.0f).margin(1e-3));
    REQUIRE(transformation.getDistortionCoefficients() == distortion);
    REQUIRE(transformation.getSourceSize() == std::pair<size_t, size_t>(mode.width, mode.height));

    const auto extrinsics = transformation.getExtrinsics();
    REQUIRE(extrinsics.toCameraSocket == origin);
    REQUIRE(extrinsics.getRotationMatrix() == identity);
    REQUIRE(extrinsics.getTranslationVector(false) == translation);
    REQUIRE(extrinsics.getTranslationVector(true) == specTranslation);
}
