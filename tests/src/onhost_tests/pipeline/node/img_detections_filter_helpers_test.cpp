#include "img_detections_filter_test_fixtures.hpp"

using namespace filtertest;

TEST_CASE("ImgDetectionsFilter H-1: transformation fixtures", "[ImgDetectionsFilter][control]") {
    REQUIRE(transformation(8, 4).getSize() == std::make_pair(std::size_t{8}, std::size_t{4}));
    REQUIRE(scaled(transformation(), .5f).getSize() == std::make_pair(std::size_t{256}, std::size_t{256}));
    REQUIRE(cropped(transformation(8, 4), 4, 0, 4, 4).getSize() == std::make_pair(std::size_t{4}, std::size_t{4}));
    REQUIRE(transformation().isValid());
    REQUIRE_FALSE(dai::ImgTransformation().isValid());
}

TEST_CASE("ImgDetectionsFilter H-2: fixture rotation direction", "[ImgDetectionsFilter][control]") {
    SECTION("90 degrees") {
        const auto p = transformation(512, 512, rotation(90)).remapPointTo(transformation(), dai::Point2f(300, 200, false));
        REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(312, .01));
        REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(300, .01));
    }
    SECTION("30 degrees") {
        const auto p = transformation(512, 512, rotation(30)).remapPointTo(transformation(), dai::Point2f(356, 256, false));
        REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(342.60, .01));
        REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(306, .01));
    }
}

TEST_CASE("ImgDetectionsFilter H-3: mask text roundtrip", "[ImgDetectionsFilter][control]") {
    const std::string grid = "0 1 . 254\n. 2 3 0";
    const auto bytes = maskBytes(grid);
    REQUIRE(bytes[2] == 255);
    REQUIRE(maskGrid(bytes, 4) == grid);
}

TEST_CASE("ImgDetectionsFilter H-4: box normalization", "[ImgDetectionsFilter][control]") {
    const auto d = detection(transformation(), {128, 128, 64, 64});
    const auto box = d.getBoundingBox();
    REQUIRE(box.isNormalized());
    REQUIRE(box.center.x == .25f);
    REQUIRE(box.center.y == .25f);
    REQUIRE(box.size.width == .125f);
    REQUIRE(box.size.height == .125f);
    requireDetection(d, 512, 512, {128, 128, 64, 64});
}
