#include "img_detections_filter_test_fixtures.hpp"
#include "pipeline/utilities/Alignment/AlignmentUtilities.hpp"

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

TEST_CASE("ImgTransformation remapRectTo preserves the fitter's angle and side ordering", "[ImgTransformation]") {
    const auto t = transformation(512, 256);
    const bool normalized = GENERATE(false, true);
    const dai::RotatedRect pixels(dai::Point2f(256, 128, false), dai::Size2f(100, 40, false), 60);
    const auto result = t.remapRectTo(t, normalized ? pixels.normalize(512, 256) : pixels);
    REQUIRE(result.isNormalized() == normalized);
    const auto box = result.denormalize(512, 256);
    REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(256, .001));
    REQUIRE_THAT(box.center.y, Catch::Matchers::WithinAbs(128, .001));
    REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(100, .001));
    REQUIRE_THAT(box.size.height, Catch::Matchers::WithinAbs(40, .001));
    REQUIRE_THAT(box.angle, Catch::Matchers::WithinAbs(60, .001));
}

TEST_CASE("rayToPixel distinguishes unprojectable pinhole rays from panorama directions", "[ImgTransformation]") {
    auto target = transformation();
    const float z = GENERATE(-1.0f, 0.0f, 1.0f);
    const auto point = rayToPixel({1, 0, z}, target);
    if(z <= 0) {
        REQUIRE(std::isnan(point.x));
        REQUIRE(std::isnan(point.y));
    } else {
        REQUIRE_THAT(point.x, Catch::Matchers::WithinAbs(512, .001));
        REQUIRE_THAT(point.y, Catch::Matchers::WithinAbs(256, .001));
    }
    const auto model = GENERATE(dai::CameraModel::Equirectangular, dai::CameraModel::Cylindrical);
    target.setDistortionModel(model);
    const auto panorama = rayToPixel({1, 0, z}, target);
    REQUIRE(std::isfinite(panorama.x));
    REQUIRE(std::isfinite(panorama.y));
    REQUIRE_THAT(panorama.x, Catch::Matchers::WithinAbs(256 + 256 * std::atan2(1.0f, z), .001));
    REQUIRE_THAT(panorama.y, Catch::Matchers::WithinAbs(256, .001));
}
