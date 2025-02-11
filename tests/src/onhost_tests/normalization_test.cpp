#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/common/Rect.hpp>

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
    dai::ImgTransformation transformFrom(2000, 1000);
    dai::ImgTransformation transformTo(2000, 1000);
    transformTo.addScale(2, 2);
    transformTo.addCrop(0, 0, 2000, 1000);
    rect = transformFrom.remapRectTo(transformTo, rect);
    REQUIRE(rect.isNormalized());
    REQUIRE(float_eq(rect.center.x, 0.4));
    REQUIRE(float_eq(rect.center.y, 0.6));
    REQUIRE(float_eq(rect.size.width, 0.8));
    REQUIRE(float_eq(rect.size.height, 1.0));
}
