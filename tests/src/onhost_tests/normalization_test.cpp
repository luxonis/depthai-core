#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <depthai/common/Rect.hpp>

#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/RotatedRect.hpp"

bool float_eq(float a, float b) {
    return std::abs(a - b) < 1e-6;
}

TEST_CASE("Rect normalization") {
    dai::Rect rect(100, 100, 600, 400);
    REQUIRE(!rect.isNormalized());
    rect = rect.normalize(2000, 1000);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.x, 0.05));
    REQUIRE(float_eq(rect.y, 0.1));
    REQUIRE(float_eq(rect.width, 0.3));
    REQUIRE(float_eq(rect.height, 0.4));
    rect = rect.denormalize(2000, 1000);
    REQUIRE(!rect.isNormalized());
    REQUIRE(rect.x == 100);
    REQUIRE(rect.y == 100);
    REQUIRE(rect.width == 600);
    REQUIRE(rect.height == 400);
}

TEST_CASE("RotatedRect normalization") {
    dai::RotatedRect rect(dai::Rect(100 - 300, 100 - 200, 600, 400), 45);
    REQUIRE(!rect.isNormalized());
    rect = rect.normalize(2000, 1000);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.center.x, 0.05));
    REQUIRE(float_eq(rect.center.y, 0.1));
    REQUIRE(float_eq(rect.size.width, 0.3));
    REQUIRE(float_eq(rect.size.height, 0.4));
    rect = rect.denormalize(2000, 1000);
    REQUIRE(!rect.isNormalized());
    REQUIRE(rect.center.x == 100);
    REQUIRE(rect.center.y == 100);
    REQUIRE(rect.size.width == 600);
    REQUIRE(rect.size.height == 400);
}

TEST_CASE("Normalized point remap") {
    dai::Point2f pt(0.2, 0.3, true);
    REQUIRE(pt.isNormalized());
    dai::ImgTransformation transformFrom(2000, 1000);
    dai::ImgTransformation transformTo(2000, 1000);
    dai::Extrinsics extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A};
    transformFrom.setExtrinsics(extrinsics);
    transformTo.setExtrinsics(extrinsics);
    transformTo.addScale(2, 2);
    transformTo.addCrop(0, 0, 2000, 1000);
    pt = transformFrom.remapPointTo(transformTo, pt);
    REQUIRE(pt.isNormalized());
    REQUIRE(float_eq(pt.x, 0.4));
    REQUIRE(float_eq(pt.y, 0.6));
}

TEST_CASE("Normalized rect remap") {
    dai::RotatedRect rect(dai::Point2f(0.2, 0.3, true), dai::Size2f(0.4, 0.5, true), 45);
    REQUIRE(rect.isNormalized());
    dai::Extrinsics extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A};
    dai::ImgTransformation transformFrom(2000, 1000);
    dai::ImgTransformation transformTo(2000, 1000);
    transformFrom.setExtrinsics(extrinsics);
    transformTo.setExtrinsics(extrinsics);

    transformTo.addScale(2, 2);
    transformTo.addCrop(0, 0, 2000, 1000);
    rect = transformFrom.remapRectTo(transformTo, rect);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.center.x, 0.4));
    REQUIRE(float_eq(rect.center.y, 0.6));
    REQUIRE(float_eq(rect.size.width, 0.8));
    REQUIRE(float_eq(rect.size.height, 1.0));
}

TEST_CASE("Normalized point outputs stay normalized outside unit range") {
    dai::Extrinsics extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A};
    dai::ImgTransformation transformFrom(2000, 1000);
    dai::ImgTransformation transformTo(2000, 1000);
    transformFrom.setExtrinsics(extrinsics);
    transformTo.setExtrinsics(extrinsics);
    transformTo.addScale(2, 2);
    transformTo.addCrop(0, 0, 2000, 1000);

    SECTION("remapPointTo preserves normalized flag for values above 1") {
        dai::Point2f pt(0.75, 0.75, true);
        const auto remapped = transformFrom.remapPointTo(transformTo, pt);

        REQUIRE(float_eq(remapped.x, 1.5));
        REQUIRE(float_eq(remapped.y, 1.5));
        REQUIRE(remapped.normalized);
        REQUIRE(remapped.hasNormalized);
        REQUIRE(remapped.isNormalized());
    }

    SECTION("projectPointTo preserves normalized flag for values above 1") {
        dai::Point2f pt(0.75, 0.75, true);
        const auto projected = transformFrom.projectPointTo(transformTo, pt, 1000.0f);

        REQUIRE(float_eq(projected.x, 1.5));
        REQUIRE(float_eq(projected.y, 1.5));
        REQUIRE(projected.normalized);
        REQUIRE(projected.hasNormalized);
        REQUIRE(projected.isNormalized());
    }
}

TEST_CASE("RotatedRect getPoints keeps the normalization flag") {
    SECTION("normalized rect produces normalized corner points") {
        dai::RotatedRect rect(dai::Point2f(0.5, 0.5, true), dai::Size2f(0.4, 0.2, true), 30);
        REQUIRE(rect.isNormalized());
        for(const auto& point : rect.getPoints()) {
            REQUIRE(point.hasNormalized);
            REQUIRE(point.isNormalized());
        }
    }

    SECTION("denormalized rect produces denormalized corner points") {
        dai::RotatedRect rect(dai::Point2f(320, 240, false), dai::Size2f(200, 100, false), 30);
        REQUIRE_FALSE(rect.isNormalized());
        for(const auto& point : rect.getPoints()) {
            REQUIRE(point.hasNormalized);
            REQUIRE_FALSE(point.isNormalized());
        }
    }
}

TEST_CASE("RotatedRect getOuterRectInFrame rotates normalized rects in pixels") {
    const dai::RotatedRect pixels(dai::Point2f(400, 250, false), dai::Size2f(200, 100, false), 30);
    const auto expected = pixels.getOuterRect();
    REQUIRE(pixels.getOuterRectInFrame(800, 500) == expected);

    const auto bounds = pixels.normalize(800, 500).getOuterRectInFrame(800, 500);
    const std::array<float, 4> scale{800, 500, 800, 500};
    for(std::size_t i = 0; i < bounds.size(); ++i) {
        REQUIRE_THAT(bounds[i] * scale[i], Catch::Matchers::WithinAbs(expected[i], 1e-3));
    }
}

dai::ImgTransformation makeTransformation(unsigned int width, unsigned int height) {
    dai::ImgTransformation transformation(width, height);
    transformation.setExtrinsics(dai::Extrinsics{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A});
    return transformation;
}

TEST_CASE("Rect remap returns the form nearest to the source angle") {
    const float angle = GENERATE(-0.001f, 0.0f, 0.001f, 30.0f, 45.0f, 89.0f, 90.0f, 91.0f, 170.0f, 350.0f);
    const float rotation = GENERATE(-0.001f, 0.0f, 0.001f);
    const bool normalized = GENERATE(false, true);
    CAPTURE(angle, rotation, normalized);

    const auto source = makeTransformation(800, 500);
    auto target = source;
    target.addRotation(rotation, dai::Point2f(400, 250, false));
    dai::RotatedRect rect(dai::Point2f(400, 250, false), dai::Size2f(200, 100, false), angle);
    if(normalized) rect = rect.normalize(800, 500);

    auto result = source.remapRectTo(target, rect);
    REQUIRE(result.isNormalized() == normalized);
    result = result.denormalize(800, 500);
    // The fitter wraps the angle to [0, 90); the result must keep the angle and the sides of the source.
    REQUIRE_THAT(result.angle, Catch::Matchers::WithinAbs(angle + rotation, 1e-2));
    REQUIRE_THAT(result.size.width, Catch::Matchers::WithinAbs(200, 0.1));
    REQUIRE_THAT(result.size.height, Catch::Matchers::WithinAbs(100, 0.1));
}

TEST_CASE("Rect remap through a horizontal flip returns the form nearest to the source angle") {
    // The mirror image of a 200x100 box at 30 deg is 200x100 at -30 deg, which is 100x200 at 60 deg (nearer to 30 deg).
    const auto [angle, expectedAngle, expectedWidth, expectedHeight] =
        GENERATE(table<float, float, float, float>({{0.0f, 0.0f, 200.0f, 100.0f}, {30.0f, 60.0f, 100.0f, 200.0f}, {91.0f, 89.0f, 200.0f, 100.0f}}));
    CAPTURE(angle);

    const auto source = makeTransformation(800, 500);
    auto target = source;
    target.addFlipHorizontal();
    const dai::RotatedRect rect(dai::Point2f(300, 200, false), dai::Size2f(200, 100, false), angle);

    const auto result = source.remapRectTo(target, rect);
    REQUIRE_THAT(result.angle, Catch::Matchers::WithinAbs(expectedAngle, 1e-2));
    REQUIRE_THAT(result.size.width, Catch::Matchers::WithinAbs(expectedWidth, 0.1));
    REQUIRE_THAT(result.size.height, Catch::Matchers::WithinAbs(expectedHeight, 0.1));
    // The geometry is the mirror image of the source.
    const auto sourceOuter = rect.getOuterRect();
    const std::array<float, 4> expectedOuter{800 - sourceOuter[2], sourceOuter[1], 800 - sourceOuter[0], sourceOuter[3]};
    const auto outer = result.getOuterRect();
    for(std::size_t i = 0; i < outer.size(); ++i) {
        REQUIRE_THAT(outer[i], Catch::Matchers::WithinAbs(expectedOuter[i], 0.1));
    }
}

TEST_CASE("Rect remap after a 90 degree frame rotation keeps an axis-aligned box at 0 degrees") {
    const float rotation = GENERATE(90.0f, -90.0f, 270.0f);
    const bool normalized = GENERATE(false, true);
    CAPTURE(rotation, normalized);

    const auto source = makeTransformation(800, 500);
    auto target = source;
    target.addRotation(rotation, dai::Point2f(400, 250, false));
    dai::RotatedRect rect(dai::Point2f(400, 250, false), dai::Size2f(200, 100, false), 0.0f);
    if(normalized) rect = rect.normalize(800, 500);

    const auto result = source.remapRectTo(target, rect);
    // The width is the side between corners 0 and 1 of the result: 100 px wide and 200 px high at 0 deg.
    const auto pixels = result.denormalize(800, 500);
    REQUIRE_THAT(pixels.angle, Catch::Matchers::WithinAbs(0, 1e-2));
    REQUIRE_THAT(pixels.size.width, Catch::Matchers::WithinAbs(100, 0.1));
    REQUIRE_THAT(pixels.size.height, Catch::Matchers::WithinAbs(200, 0.1));
    // At 0 deg, getOuterRect() of the normalized result is also correct.
    const std::array<float, 4> expectedOuter{350, 150, 450, 350};
    const std::array<float, 4> scale = normalized ? std::array<float, 4>{800, 500, 800, 500} : std::array<float, 4>{1, 1, 1, 1};
    const auto outer = result.getOuterRect();
    for(std::size_t i = 0; i < outer.size(); ++i) {
        REQUIRE_THAT(outer[i] * scale[i], Catch::Matchers::WithinAbs(expectedOuter[i], 0.1));
    }
}

TEST_CASE("transformRect and invTransformRect return the form nearest to the source angle") {
    // {frame rotation, expected angle after transformRect, expected width, expected height}
    const auto [rotation, expectedAngle, expectedWidth, expectedHeight] =
        GENERATE(table<float, float, float, float>({{-0.001f, -0.001f, 200.0f, 100.0f}, {0.001f, 0.001f, 200.0f, 100.0f}, {90.0f, 0.0f, 100.0f, 200.0f}}));
    CAPTURE(rotation);

    dai::ImgTransformation transformation(800, 500);
    transformation.addRotation(rotation, dai::Point2f(400, 250, false));
    const dai::RotatedRect rect(dai::Point2f(400, 250, false), dai::Size2f(200, 100, false), 0.0f);

    const auto transformed = transformation.transformRect(rect);
    REQUIRE_THAT(transformed.angle, Catch::Matchers::WithinAbs(expectedAngle, 1e-2));
    REQUIRE_THAT(transformed.size.width, Catch::Matchers::WithinAbs(expectedWidth, 0.1));
    REQUIRE_THAT(transformed.size.height, Catch::Matchers::WithinAbs(expectedHeight, 0.1));

    const auto inverse = transformation.invTransformRect(rect);
    REQUIRE_THAT(inverse.angle, Catch::Matchers::WithinAbs(-expectedAngle, 1e-2));
    REQUIRE_THAT(inverse.size.width, Catch::Matchers::WithinAbs(expectedWidth, 0.1));
    REQUIRE_THAT(inverse.size.height, Catch::Matchers::WithinAbs(expectedHeight, 0.1));
}

TEST_CASE("Rect projection returns the form nearest to the source angle and does not change its argument") {
    const auto [rotation, expectedAngle, expectedWidth, expectedHeight] =
        GENERATE(table<float, float, float, float>({{-0.001f, -0.001f, 200.0f, 100.0f}, {0.001f, 0.001f, 200.0f, 100.0f}, {90.0f, 0.0f, 100.0f, 200.0f}}));
    CAPTURE(rotation);

    const auto source = makeTransformation(800, 500);
    auto target = source;
    target.addRotation(rotation, dai::Point2f(400, 250, false));
    const dai::RotatedRect original = dai::RotatedRect(dai::Point2f(400, 250, false), dai::Size2f(200, 100, false), 0.0f).normalize(800, 500);
    dai::RotatedRect rect = original;

    const auto projected = source.projectRectTo(target, rect, 1000.0f);
    REQUIRE(projected.isNormalized());
    const auto pixels = projected.denormalize(800, 500);
    REQUIRE_THAT(pixels.angle, Catch::Matchers::WithinAbs(expectedAngle, 1e-2));
    REQUIRE_THAT(pixels.size.width, Catch::Matchers::WithinAbs(expectedWidth, 0.1));
    REQUIRE_THAT(pixels.size.height, Catch::Matchers::WithinAbs(expectedHeight, 0.1));

    REQUIRE(rect.isNormalized());
    REQUIRE(rect.center.x == original.center.x);
    REQUIRE(rect.center.y == original.center.y);
    REQUIRE(rect.size.width == original.size.width);
    REQUIRE(rect.size.height == original.size.height);
}
