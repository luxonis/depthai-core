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

TEST_CASE("ImgDetectionsFilter splits boxes crossing the longitude join", "[ImgDetectionsFilter][panorama]") {
    const auto model = GENERATE(dai::CameraModel::Equirectangular, dai::CameraModel::Cylindrical);
    const float yaw = GENERATE(-179.f, 179.f);
    const float homogeneousScale = GENERATE(1.f, 2.f, -2.f);
    FilterSettings s;
    s.mode = GENERATE(Mode::OFF, Mode::NMS, Mode::AVERAGE);
    const float focal = 256 / (2 * std::acos(-1.0f));
    s.reference = transformation(256, 128, IDENTITY, model, {}, focal);
    s.reference->addTransformation({{{homogeneousScale, 0, 0}, {0, homogeneousScale, 0}, {0, 0, homogeneousScale}}});
    const auto source = transformation(512, 512, rotation(yaw, true));
    const auto input = message(source, {{256, 256, 80, 120, .9f, 5, 0, "person", {{236, 256}, {276, 256}}}});
    const MessageSnapshot snapshot(*input);
    FilterHarness h(s, {"a", "b"});
    h.pipeline.start();
    h.send(input, "a");
    h.send(message(source, {{256, 256, 80, 120, .7f, 5, 0, "person", {{236, 256}, {276, 256}}}}), "b");
    const auto out = h.receive();
    REQUIRE(out->detections.size() == (*s.mode == Mode::OFF ? 4 : 2));
    const float halfWidth = focal * std::atan(40.f / 256);
    const float centerOffset = yaw > 0 ? -256.f / 360 : 256.f / 360;
    for(std::size_t i = 0; i < out->detections.size(); ++i) {
        const auto& detection = out->detections[i];
        const bool left = i % 2 == 0;
        const float minX = left ? 0 : 256 + centerOffset - halfWidth;
        const float maxX = left ? centerOffset + halfWidth : 256;
        const auto box = detection.getBoundingBox().denormalize(256, 128);
        REQUIRE(box.angle == 0);
        REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs((minX + maxX) / 2, .02));
        REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(maxX - minX, .02));
        REQUIRE(detection.label == 5);
        REQUIRE(detection.labelName == "person");
        REQUIRE(detection.confidence == (i < 2 ? .9f : .7f));
        REQUIRE(detection.xmin == Catch::Approx(minX / 256).margin(1e-4));
        REQUIRE(detection.xmax == Catch::Approx(maxX / 256).margin(1e-4));
        const auto points = detection.getKeypoints();
        REQUIRE(points.size() == 2);
        REQUIRE(points[left ? 0 : 1].confidence == 0);
        REQUIRE(points[left ? 1 : 0].confidence == .9f);
        REQUIRE(points[left ? 1 : 0].imageCoordinates.x >= detection.xmin);
        REQUIRE(points[left ? 1 : 0].imageCoordinates.x <= detection.xmax);
    }
    requireMetadata(*out, *input);
    snapshot.requireEqual(*input);
}

namespace {
// Cameras looking at longitude 180, which is the join at the canvas edges of degreePanorama().
const Rotation BACKWARD = {{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}}}, BACKWARD_UPSIDE_DOWN = {{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};

// One pixel is one degree.
dai::ImgTransformation degreePanorama() {
    return transformation(360, 180, IDENTITY, dai::CameraModel::Equirectangular, {}, 360 / (2 * std::acos(-1.0f)));
}

// A 120 px high box between two angles from the optical axis of a 512 px camera with a 256 px focal length.
ExpectedDetection span(float from, float to, float confidence = .9f) {
    const auto column = [](float degrees) { return 256 + 256 * std::tan(degrees * std::acos(-1.0f) / 180); };
    return {(column(from) + column(to)) / 2, 256, column(to) - column(from), 120, confidence, 5};
}

void requireColumns(const dai::ImgDetections& out, const std::vector<std::array<float, 2>>& columns) {
    REQUIRE(out.detections.size() == columns.size());
    for(std::size_t i = 0; i < columns.size(); ++i) {
        CAPTURE(i);
        const auto box = out.detections[i].getBoundingBox().denormalize(360, 180);
        REQUIRE_THAT(box.center.x - box.size.width / 2, Catch::Matchers::WithinAbs(columns[i][0], .02));
        REQUIRE_THAT(box.center.x + box.size.width / 2, Catch::Matchers::WithinAbs(columns[i][1], .02));
    }
}
}  // namespace

TEST_CASE("ImgDetectionsFilter finds duplicates of whole boxes across the longitude join", "[ImgDetectionsFilter][panorama]") {
    // Camera a sees longitudes 172 to 192 and camera b 178 to 198: the boxes have IoU 0.54, their parts left of the join only 0.25.
    const bool upsideDown = GENERATE(false, true);  // Unwraps the box of camera b one period away from the box of camera a.
    FilterSettings s;
    s.reference = degreePanorama();
    s.mode = GENERATE(Mode::NMS, Mode::AVERAGE);
    FilterHarness h(s, {"a", "b"});
    h.pipeline.start();
    h.send(message(transformation(512, 512, BACKWARD), {span(-8, 12, .9f)}), "a");
    h.send(upsideDown ? message(transformation(512, 512, BACKWARD_UPSIDE_DOWN), {span(-18, 2, .8f)})
                      : message(transformation(512, 512, BACKWARD), {span(-2, 18, .8f)}),
           "b");
    // NMS keeps camera a. AVERAGE weights the box centers 182 and 188 by confidence.
    const float minX = *s.mode == Mode::NMS ? 352 : 354.824f;
    requireColumns(*h.receive(), {{0, minX - 340}, {minX, 360}});
}

TEST_CASE("ImgDetectionsFilter applies size limits and ROI to whole boxes at the longitude join", "[ImgDetectionsFilter][panorama]") {
    // The box spans longitudes 172 to 192: 20 px, split into parts 12 and 8 px wide.
    FilterSettings s;
    s.reference = degreePanorama();
    std::vector<std::array<float, 2>> expected;
    SECTION("minimum width counts both parts") {
        s.minWidth = 15;
        expected = {{0, 12}, {352, 360}};
    }
    SECTION("maximum width counts both parts") {
        s.maxWidth = 15;
    }
    SECTION("ROI must contain every part") {
        s.roi = dai::Rect(300, 0, 60, 180, false);
    }
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, BACKWARD), {span(-8, 12)}));
    requireColumns(*h.receive(), expected);
}

TEST_CASE("ImgDetectionsFilter drops fragments narrower than one pixel at the longitude join", "[ImgDetectionsFilter][panorama]") {
    FilterSettings s;
    s.reference = degreePanorama();
    ExpectedDetection box{};
    std::vector<std::array<float, 2>> expected;
    SECTION("sliver") {
        box = span(-10, .5f);
        expected = {{350, 360}};
    }
    SECTION("one pixel fragment") {
        box = span(-10, 1.5f);
        expected = {{0, 1.5f}, {350, 360}};
    }
    SECTION("box narrower than two pixels keeps its wider part") {
        box = span(-.3f, .6f);
        expected = {{0, .6f}};
    }
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, BACKWARD), {box}));
    requireColumns(*h.receive(), expected);
}

TEST_CASE("ImgDetectionsFilter keeps wrapped mask indices with their fragments", "[ImgDetectionsFilter][panorama]") {
    const auto model = GENERATE(dai::CameraModel::Equirectangular, dai::CameraModel::Cylindrical);
    FilterSettings s;
    s.reference = transformation(256, 128, IDENTITY, model, {}, 256 / (2 * std::acos(-1.0f)));
    int visibleSide = 0;
    SECTION("both fragments") {}
    SECTION("left fragment visible") {
        visibleSide = -1;
    }
    SECTION("right fragment visible") {
        visibleSide = 1;
    }
    SECTION("maximum count removes a fragment") {
        s.maxDetections = 1;
    }
    FilterHarness h(s);
    h.linkMask();
    auto mask = std::make_shared<dai::ImgFrame>();
    mask->setType(dai::ImgFrame::Type::GRAY8);
    mask->setSize(256, 128);
    mask->setTransformation(*s.reference);
    std::vector<std::uint8_t> visible(256 * 128, 255);
    for(std::size_t i = 0; i < visible.size(); ++i)
        if((visibleSide < 0 && i % 256 >= 128) || (visibleSide > 0 && i % 256 < 128)) visible[i] = 0;
    mask->setData(visible);
    const auto input = message(transformation(512, 512, rotation(179, true)), {{256, 256, 80, 120}});
    input->setSegmentationMask(std::vector<std::uint8_t>(8 * 8, 0), 8, 8);
    h.pipeline.start();
    h.sendMask(mask);
    h.send(input);
    const auto out = h.receive();
    REQUIRE(out->detections.size() == (visibleSide != 0 || s.maxDetections ? 1 : 2));
    REQUIRE(out->getMaskData().has_value());
    REQUIRE(out->getSegmentationMaskWidth() == 256);
    REQUIRE(out->getSegmentationMaskHeight() == 128);
    const auto bytes = *out->getMaskData();
    std::vector<std::size_t> counts(out->detections.size(), 0);
    for(std::size_t i = 0; i < bytes.size(); ++i) {
        if(bytes[i] == 255) continue;
        REQUIRE(bytes[i] < out->detections.size());
        ++counts[bytes[i]];
        const auto& detection = out->detections[bytes[i]];
        const float x = (i % 256 + .5f) / 256;
        REQUIRE(x >= detection.xmin);
        REQUIRE(x <= detection.xmax);
    }
    for(const auto count : counts) REQUIRE(count > 0);
    if(visibleSide != 0) REQUIRE((out->detections[0].getBoundingBox().center.x < .5f) == (visibleSide < 0));
}

TEST_CASE("ImgDetectionsFilter locates the longitude join after crop and scale", "[ImgDetectionsFilter][panorama]") {
    const auto model = GENERATE(dai::CameraModel::Equirectangular, dai::CameraModel::Cylindrical);
    const int cropX = GENERATE(0, 128);
    FilterSettings s;
    const float focal = 256 / (2 * std::acos(-1.0f));
    s.reference = scaled(cropped(transformation(256, 128, IDENTITY, model, {}, focal), cropX, 0, 128, 128), 2);
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, rotation(179, true)), {{256, 256, 80, 120}}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const auto box = out->detections[0].getBoundingBox().denormalize(256, 256);
    const float halfWidth = focal * std::atan(40.f / 256);
    const float width = 2 * (halfWidth + (cropX == 0 ? -256.f / 360 : 256.f / 360));
    REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(width, .02));
    REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(cropX == 0 ? width / 2 : 256 - width / 2, .02));
}

TEST_CASE("ImgDetectionsFilter preserves bounds in a rotated panorama", "[ImgDetectionsFilter][panorama]") {
    FilterSettings s;
    s.reference = transformation(256, 128, IDENTITY, dai::CameraModel::Equirectangular, {}, 256 / (2 * std::acos(-1.0f)));
    s.reference->addRotation(90, dai::Point2f(128, 64, false));
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{256, 256, 80, 120}}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const float focal = 256 / (2 * std::acos(-1.0f));
    const auto box = out->detections[0].getBoundingBox().denormalize(256, 128);
    REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(128, .02));
    REQUIRE_THAT(box.center.y, Catch::Matchers::WithinAbs(64, .02));
    REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(2 * focal * std::atan(60.f / 256), .02));
    REQUIRE_THAT(box.size.height, Catch::Matchers::WithinAbs(2 * focal * std::atan(40.f / 256), .02));
}

TEST_CASE("ImgDetectionsFilter unions original seam masks after averaging", "[ImgDetectionsFilter][panorama]") {
    const auto model = GENERATE(dai::CameraModel::Equirectangular, dai::CameraModel::Cylindrical);
    FilterSettings s;
    s.mode = GENERATE(Mode::OFF, Mode::NMS, Mode::AVERAGE);
    s.reference = transformation(256, 128, IDENTITY, model, {}, 256 / (2 * std::acos(-1.0f)));
    const auto source = transformation(512, 512, rotation(179, true));
    const auto narrow = message(source, {{256, 256, 64, 120}});
    const auto wide = message(source, {{256, 256, 128, 120}});
    narrow->setSegmentationMask(std::vector<std::uint8_t>(8 * 8, 0), 8, 8);
    wide->setSegmentationMask(std::vector<std::uint8_t>(8 * 8, 0), 8, 8);
    FilterHarness h(s, {"a", "b"});
    h.pipeline.start();
    h.send(narrow, "a");
    h.send(wide, "b");
    const auto out = h.receive();
    REQUIRE(out->detections.size() == (*s.mode == Mode::OFF ? 4 : 2));
    REQUIRE(out->getMaskData().has_value());
    const auto bytes = *out->getMaskData();
    REQUIRE(bytes.size() == 256 * 128);
    const int leftIndex = *s.mode == Mode::OFF ? 2 : *s.mode == Mode::AVERAGE ? 0 : 255;
    const int rightIndex = leftIndex == 255 ? 255 : leftIndex + 1;
    // These pixels belong only to the wider member and lie beyond the averaged box on both sides of the join.
    REQUIRE(bytes[64 * 256 + 8] == leftIndex);
    REQUIRE(bytes[64 * 256 + 246] == rightIndex);
    if(*s.mode == Mode::AVERAGE) {
        REQUIRE(out->detections[0].xmax < 8.5f / 256);
        REQUIRE(out->detections[1].xmin > 246.5f / 256);
    }
}

TEST_CASE("ImgDetectionsFilter keeps full longitude coverage around a pole", "[ImgDetectionsFilter][panorama]") {
    // One pixel is one degree. The box corners lie 35.264 degrees from the pole, its edge midpoints 26.565 degrees.
    float sign = 1, side = 256, minY = 0, maxY = 35.264f;
    bool crop = false;
    auto model = dai::CameraModel::Equirectangular;
    SECTION("north pole") {}
    SECTION("south pole") {
        sign = -1;
        minY = 144.736f;
        maxY = 180;
    }
    SECTION("crop inside the former longitude gap") {
        crop = true;
        minY = -33;
        maxY = 2.264f;
    }
    SECTION("cylindrical pole reaches the canvas edge") {
        // The sampled edges span rows 16.661 to 38.142; the pole lies at infinity above them.
        model = dai::CameraModel::Cylindrical;
        side = 400;
        maxY = 38.142f;
    }
    FilterSettings s;
    s.reference = transformation(360, 180, IDENTITY, model, {}, 360 / (2 * std::acos(-1.0f)));
    if(crop) s.reference->addCrop(226, 33, 1, 3);
    const Rotation pitch = {{{1, 0, 0}, {0, 0, -sign}, {0, sign, 0}}};
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, pitch), {{256, 256, side, side}}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const auto size = s.reference->getSize();
    const auto box = out->detections[0].getBoundingBox().denormalize(size.first, size.second);
    REQUIRE_THAT(box.center.x, Catch::Matchers::WithinAbs(crop ? -46 : 180, .01));
    REQUIRE_THAT(box.size.width, Catch::Matchers::WithinAbs(360, .01));
    REQUIRE_THAT(box.center.y, Catch::Matchers::WithinAbs((minY + maxY) / 2, .01));
    REQUIRE_THAT(box.size.height, Catch::Matchers::WithinAbs(maxY - minY, .01));
}
