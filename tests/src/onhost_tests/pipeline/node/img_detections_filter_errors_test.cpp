#include "img_detections_filter_test_harness.hpp"

using namespace filtertest;

TEST_CASE("ImgDetectionsFilter X-1: remap requires input transformation", "[ImgDetectionsFilter][ER-5]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s);
    auto input = message();
    input->transformation.reset();
    h.pipeline.start();
    h.send(input);
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-2: empty input still requires transformation", "[ImgDetectionsFilter][ER-5]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s, {"a", "b"});
    auto input = message(transformation(), {});
    input->transformation.reset();
    h.pipeline.start();
    h.send(message(), "a");
    h.send(input, "b");
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-3: pixel limits require transformation", "[ImgDetectionsFilter][ER-5]") {
    FilterSettings s;
    s.minArea = 10;
    FilterHarness h(s);
    auto input = message();
    input->transformation.reset();
    h.pipeline.start();
    h.send(input);
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-3b: invalid transformation with pixel limits", "[ImgDetectionsFilter][ER-5]") {
    FilterSettings s;
    s.minArea = 10;
    FilterHarness h(s);
    auto input = message();
    input->setTransformation(dai::ImgTransformation());
    h.pipeline.start();
    h.send(input);
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-4: coordinate systems must match", "[ImgDetectionsFilter][ER-6]") {
    auto source = transformation(), target = transformation();
    auto e = source.getExtrinsics();
    e.toDeviceId = "dev-a";
    source.setExtrinsics(e);
    e.toDeviceId = "dev-b";
    target.setExtrinsics(e);
    FilterSettings s;
    s.reference = target;
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(source));
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-5: width filter requires a box", "[ImgDetectionsFilter][ER-10]") {
    FilterSettings s;
    s.minWidth = 10;
    FilterHarness h(s);
    auto input = message(transformation(), {});
    dai::ImgDetection d;
    d.label = 1;
    d.confidence = .9f;
    input->detections.push_back(d);
    h.pipeline.start();
    h.send(input);
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-5b: remap requires a box", "[ImgDetectionsFilter][ER-10]") {
    FilterSettings s;
    s.reference = scaled(transformation(), .5f);
    FilterHarness h(s);
    auto input = message(transformation(), {});
    dai::ImgDetection d;
    d.label = 1;
    d.confidence = .9f;
    input->detections.push_back(d);
    h.pipeline.start();
    h.send(input);
    h.requireStopped();
}
TEST_CASE("ImgDetectionsFilter X-5c: confidence rejects missing box before geometry", "[ImgDetectionsFilter][ER-10][ST-1]") {
    FilterSettings s;
    s.minWidth = 10;
    s.minConfidence = .5f;
    FilterHarness h(s);
    auto input = message();
    dai::ImgDetection d;
    d.confidence = .3f;
    input->detections.insert(input->detections.begin(), d);
    h.pipeline.start();
    for(int i = 0; i < 2; ++i) {
        h.send(input);
        requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 1}});
    }
    REQUIRE(h.pipeline.isRunning());
}
TEST_CASE("ImgDetectionsFilter X-6: valid messages keep pipeline running", "[ImgDetectionsFilter][control]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s);
    h.pipeline.start();
    for(int i = 0; i < 2; ++i) {
        h.send(message());
        requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 1}});
    }
    REQUIRE(h.pipeline.isRunning());
}
