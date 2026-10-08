#include "img_detections_filter_test_harness.hpp"

using namespace filtertest;

TEST_CASE("ImgDetectionsFilter R-10: equirectangular boxes and keypoints", "[ImgDetectionsFilter][RM-3][RM-4][RM-7]") {
    FilterSettings s;
    s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Equirectangular, {}, 256);
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{256, 256, 256, 256, .9f, 5, 0, "obj", {{384, 256}, {256, 384}}}}));
    requireOutput(*h.receive(), *s.reference, {{512, 256, 237.39f, 215.31f, .9f, 5, 0, "obj", {{630.69f, 256}, {512, 374.69f}}}}, .2f, .1f);
}

TEST_CASE("ImgDetectionsFilter R-11: cylindrical boxes and keypoints", "[ImgDetectionsFilter][RM-3][RM-4][RM-7]") {
    FilterSettings s;
    s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Cylindrical, {}, 256);
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{256, 256, 256, 256, .9f, 5, 0, "obj", {{256, 384}, {384, 384}}}}));
    requireOutput(*h.receive(), *s.reference, {{512, 256, 237.39f, 228.97f, .9f, 5, 0, "obj", {{512, 384}, {630.69f, 370.49f}}}}, .2f, .1f);
}

TEST_CASE("ImgDetectionsFilter R-16: panorama keeps directions behind its axis", "[ImgDetectionsFilter][RM-3][RM-11]") {
    FilterSettings s;
    float height = 0;
    SECTION("equirectangular") {
        s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Equirectangular, {}, 256);
        height = 39.80f;
    }
    SECTION("cylindrical") {
        s.reference = transformation(1024, 512, IDENTITY, dai::CameraModel::Cylindrical, {}, 256);
        height = 39.88f;
    }
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, rotation(100, true)), {{256, 256, 40, 40, .9f, 5, 0, "obj", {{256, 256}}}}));
    requireOutput(*h.receive(), *s.reference, {{958.80f, 256, 39.92f, height, .9f, 5, 0, "obj", {{958.80f, 256}}}}, .2f, .1f);
}
