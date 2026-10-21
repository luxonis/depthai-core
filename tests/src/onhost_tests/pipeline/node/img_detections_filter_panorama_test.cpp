#include "img_detections_filter_test_harness.hpp"

using namespace filtertest;

TEST_CASE("ImgDetectionsFilter R-10: equirectangular boxes and keypoints", "[ImgDetectionsFilter][RM-3][RM-4][RM-7]") {
    FilterSettings s;
    s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Equirectangular, {}, 256);
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{256, 256, 256, 256, .9f, 5, 0, "obj", {{384, 256}, {256, 384}}}}));
    requireOutput(*h.receive(), *s.reference, {{512, 256, 237.39f, 237.39f, .9f, 5, 0, "obj", {{630.69f, 256}, {512, 374.69f}}}}, .2f, .1f);
}

TEST_CASE("ImgDetectionsFilter R-11: cylindrical boxes and keypoints", "[ImgDetectionsFilter][RM-3][RM-4][RM-7]") {
    FilterSettings s;
    s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Cylindrical, {}, 256);
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{256, 256, 256, 256, .9f, 5, 0, "obj", {{256, 384}, {384, 384}}}}));
    requireOutput(*h.receive(), *s.reference, {{512, 256, 237.39f, 256, .9f, 5, 0, "obj", {{512, 384}, {630.69f, 370.49f}}}}, .2f, .1f);
}

TEST_CASE("ImgDetectionsFilter R-16: panorama keeps directions behind its axis", "[ImgDetectionsFilter][RM-3][RM-11]") {
    FilterSettings s;
    float height = 0;
    SECTION("equirectangular") {
        s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Equirectangular, {}, 256);
        height = 39.92f;
    }
    SECTION("cylindrical") {
        s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Cylindrical, {}, 256);
        height = 40;
    }
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, rotation(100, true)), {{256, 256, 40, 40, .9f, 5, 0, "obj", {{256, 256}}}}));
    requireOutput(*h.receive(), *s.reference, {{958.80f, 256, 39.92f, height, .9f, 5, 0, "obj", {{958.80f, 256}}}}, .2f, .1f);
}

TEST_CASE("ImgDetectionsFilter R-17: panorama boxes stay axis-aligned", "[ImgDetectionsFilter][RM-3][RM-4]") {
    FilterSettings s;
    SECTION("equirectangular") {
        s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Equirectangular, {}, 256);
    }
    SECTION("cylindrical") {
        s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Cylindrical, {}, 256);
    }
    FilterHarness h(s);
    h.pipeline.start();
    // A box near the image corner of a yawed camera maps to a bent quadrilateral whose minimum-area rectangle is tilted.
    const auto source = transformation(512, 512, rotation(35, true));
    const ExpectedDetection input{140, 90, 200, 120, .9f, 5, 0, "obj"};
    h.send(message(source, {input}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const auto box = out->detections[0].getBoundingBox();
    REQUIRE(box.angle == 0);
    float minX = 1, minY = 1, maxX = 0, maxY = 0;
    const std::array<std::array<float, 2>, 4> corners = {{{input.x - input.width / 2, input.y - input.height / 2},
                                                          {input.x + input.width / 2, input.y - input.height / 2},
                                                          {input.x + input.width / 2, input.y + input.height / 2},
                                                          {input.x - input.width / 2, input.y + input.height / 2}}};
    for(std::size_t edge = 0; edge < corners.size(); ++edge) {
        const auto a = corners[edge], b = corners[(edge + 1) % corners.size()];
        for(int sample = 0; sample < 64; ++sample) {
            const float t = sample / 64.f;
            const auto mapped = source.remapPointTo(*s.reference, dai::Point2f((a[0] + t * (b[0] - a[0])) / 512, (a[1] + t * (b[1] - a[1])) / 512, true));
            minX = std::min(minX, mapped.x);
            maxX = std::max(maxX, mapped.x);
            minY = std::min(minY, mapped.y);
            maxY = std::max(maxY, mapped.y);
        }
    }
    // The mapped edges curve, so the box is wider than its corners alone and must enclose every sample.
    REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs((minX + maxX) / 2, 1e-3));
    REQUIRE_THAT(box.center.y, Catch::Matchers::WithinAbs((minY + maxY) / 2, 1e-3));
    REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(maxX - minX, 2e-3));
    REQUIRE_THAT(box.size.height, Catch::Matchers::WithinAbs(maxY - minY, 2e-3));
    REQUIRE(box.size.width > (maxX - minX) - 2e-3f);
    REQUIRE(box.size.height > (maxY - minY) - 2e-3f);
    REQUIRE(out->detections[0].xmin == Catch::Approx(box.center.x - box.size.width / 2).margin(1e-5));
    REQUIRE(out->detections[0].ymax == Catch::Approx(box.center.y + box.size.height / 2).margin(1e-5));
}
