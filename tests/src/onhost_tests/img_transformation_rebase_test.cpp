#include <catch2/catch_all.hpp>
#include <stdexcept>

#include "depthai/common/ImgTransformations.hpp"

using namespace dai;

namespace {

Extrinsics makeExtrinsics(const std::string& deviceId, CameraBoardSocket socket, const Point3f& translation, LengthUnit unit) {
    Extrinsics result({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, translation, socket, unit);
    result.toDeviceId = deviceId;
    return result;
}

}  // namespace

TEST_CASE("ImgTransformation rebases extrinsics while preserving image transformation data") {
    ImgTransformation transformation(640,
                                     480,
                                     {{{500.0f, 0.0f, 320.0f}, {0.0f, 500.0f, 240.0f}, {0.0f, 0.0f, 1.0f}}},
                                     CameraModel::Perspective,
                                     {0.1f, -0.2f},
                                     makeExtrinsics("local-device", CameraBoardSocket::CAM_A, {100.0f, 0.0f, 0.0f}, LengthUnit::CENTIMETER));
    transformation.addScale(2.0f, 1.5f);
    transformation.addCrop(10, 20, 300, 200);

    const auto matrixBefore = transformation.getMatrix();
    const auto intrinsicBefore = transformation.getSourceIntrinsicMatrix();
    const auto distortionBefore = transformation.getDistortionCoefficients();
    const auto sizeBefore = transformation.getSize();
    const auto sourceSizeBefore = transformation.getSourceSize();
    const auto cropsBefore = transformation.getSrcCrops();

    transformation.rebaseExtrinsics(makeExtrinsics("global-device", CameraBoardSocket::CAM_B, {2.0f, 0.0f, 0.0f}, LengthUnit::METER));

    const auto result = transformation.getExtrinsics();
    REQUIRE(result.toDeviceId == "global-device");
    REQUIRE(result.toCameraSocket == CameraBoardSocket::CAM_B);
    REQUIRE(result.lengthUnit == LengthUnit::METER);
    REQUIRE(result.translation.x == Catch::Approx(3.0f));
    REQUIRE(result.translation.y == Catch::Approx(0.0f));
    REQUIRE(result.translation.z == Catch::Approx(0.0f));
    REQUIRE(transformation.getMatrix() == matrixBefore);
    REQUIRE(transformation.getSourceIntrinsicMatrix() == intrinsicBefore);
    REQUIRE(transformation.getDistortionCoefficients() == distortionBefore);
    REQUIRE(transformation.getSize() == sizeBefore);
    REQUIRE(transformation.getSourceSize() == sourceSizeBefore);
    REQUIRE(transformation.getSrcCrops().size() == cropsBefore.size());
    REQUIRE(transformation.getSrcCrops()[0].center.x == Catch::Approx(cropsBefore[0].center.x));
    REQUIRE(transformation.getSrcCrops()[0].center.y == Catch::Approx(cropsBefore[0].center.y));
}

TEST_CASE("ImgTransformation rejects invalid rebasing extrinsics") {
    ImgTransformation transformation(640, 480);

    REQUIRE_THROWS_AS(transformation.rebaseExtrinsics(makeExtrinsics("global-device", CameraBoardSocket::CAM_A, {}, LengthUnit::METER)), std::invalid_argument);

    transformation.setExtrinsics(makeExtrinsics("local-device", CameraBoardSocket::CAM_A, {}, LengthUnit::METER));
    auto invalidBridge = makeExtrinsics("global-device", CameraBoardSocket::CAM_A, {}, LengthUnit::METER);
    invalidBridge.rotationMatrix[0][0] = 2.0f;
    REQUIRE_THROWS_AS(transformation.rebaseExtrinsics(invalidBridge), std::invalid_argument);
}

TEST_CASE("ImgTransformation composes rotation and translation while rebasing") {
    ImgTransformation transformation(640,
                                     480,
                                     {{{500.0f, 0.0f, 320.0f}, {0.0f, 500.0f, 240.0f}, {0.0f, 0.0f, 1.0f}}},
                                     CameraModel::Perspective,
                                     {},
                                     makeExtrinsics("local-device", CameraBoardSocket::CAM_A, {1.0f, 0.0f, 0.0f}, LengthUnit::METER));

    const auto bridge =
        Extrinsics({{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {2.0f, 0.0f, 0.0f}, CameraBoardSocket::CAM_B, LengthUnit::METER);
    auto globalBridge = bridge;
    globalBridge.toDeviceId = "global-device";

    transformation.rebaseExtrinsics(globalBridge);

    const auto result = transformation.getExtrinsics();
    REQUIRE(result.translation.x == Catch::Approx(2.0f));
    REQUIRE(result.translation.y == Catch::Approx(1.0f));
    REQUIRE(result.rotationMatrix[0][0] == Catch::Approx(0.0f));
    REQUIRE(result.rotationMatrix[0][1] == Catch::Approx(-1.0f));
    REQUIRE(result.rotationMatrix[1][0] == Catch::Approx(1.0f));
    REQUIRE(result.rotationMatrix[1][1] == Catch::Approx(0.0f));
}
