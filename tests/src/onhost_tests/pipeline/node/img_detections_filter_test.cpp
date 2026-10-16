#include <algorithm>

#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "img_detections_filter_test_cases.hpp"

using namespace filtertest;

namespace {
// Drive one complete round and check every detection against a literal oracle.
std::shared_ptr<dai::ImgDetections> checkRound(const FilterSettings& settings,
                                               const std::vector<std::shared_ptr<dai::ImgDetections>>& inputs,
                                               const std::vector<ExpectedDetection>& expected,
                                               float tolerance = 1e-4f,
                                               float angleTolerance = 1e-4f) {
    std::vector<std::string> keys;
    for(std::size_t i = 0; i < inputs.size(); ++i) keys.push_back(std::string(1, static_cast<char>('a' + i)));
    FilterHarness h(settings, keys);
    h.pipeline.start();
    for(std::size_t i = 0; i < inputs.size(); ++i) h.send(inputs[i], keys[i]);
    auto output = h.receive();
    REQUIRE(inputs.front()->getTransformation().has_value());
    requireOutput(*output, settings.reference.value_or(*inputs.front()->getTransformation()), expected, tolerance, angleTolerance);
    return output;
}

void checkReferenceRound(FilterHarness& h, bool croppedOutput, bool masked = false) {
    const auto t = transformation(8, 4);
    const std::vector<ExpectedDetection> data = {{6, 2, 2, 2, .9f}, {1, 2, 2, 2, .8f}, {4, 2, 2, 2, .7f}};
    auto input = message(t, data);
    if(masked) input->setSegmentationMask(maskBytes(". . . . . . . .\n1 1 . 2 2 0 0 .\n1 1 . 2 2 0 0 .\n. . . . . . . ."), 8, 4);
    h.send(input);
    auto output = h.receive();
    requireOutput(*output,
                  croppedOutput ? cropped(t, 4, 0, 4, 4) : t,
                  croppedOutput ? std::vector<ExpectedDetection>{{2, 2, 2, 2, .9f}, {0, 2, 2, 2, .7f}} : data,
                  .01f,
                  .01f);
    if(masked) {
        if(croppedOutput)
            requireMask(*output, 4, 4, ". . . .\n1 0 0 .\n1 0 0 .\n. . . .");
        else
            requireMask(*output, 8, 4, ". . . . . . . .\n1 1 . 2 2 0 0 .\n1 1 . 2 2 0 0 .\n. . . . . . . .");
    }
}
}  // namespace

TEST_CASE("ImgDetectionsFilter A-1: host placement and output", "[ImgDetectionsFilter][IN-1][EX-1]") {
    FilterHarness h;
    h.pipeline.start();
    REQUIRE(h.runOnHost());
    h.send(message());
    requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 1}});
}
TEST_CASE("ImgDetectionsFilter A-2: no linked keys", "[ImgDetectionsFilter][IN-3][ER-1]") {
    FilterHarness h({}, {});
    REQUIRE_THROWS(h.pipeline.start());
}
TEST_CASE("ImgDetectionsFilter A-3: multiple keys need a reference", "[ImgDetectionsFilter][ER-2][RF-9]") {
    FilterHarness h({}, {"a", "b"});
    REQUIRE_THROWS(h.pipeline.start());
}
TEST_CASE("ImgDetectionsFilter A-4: configured reference", "[ImgDetectionsFilter][RF-1]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(), message(transformation(), {{300, 300, 64, 64, .9f, 1}})}, {{100, 100, 64, 64, .9f, 1}, {300, 300, 64, 64, .9f, 1}});
}
TEST_CASE("ImgDetectionsFilter A-5: linked reference permits startup", "[ImgDetectionsFilter][RF-9]") {
    FilterHarness h({}, {"a", "b"}, false, true);
    REQUIRE_NOTHROW(h.pipeline.start());
}
TEST_CASE("ImgDetectionsFilter A-6: unlinked keys do not participate", "[ImgDetectionsFilter][IN-4]") {
    FilterHarness h({}, {"a"});
    h.accessUnlinked("b");
    h.pipeline.start();
    h.send(message(), "a");
    requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 1}});
}
TEST_CASE("ImgDetectionsFilter A-7: invalid initial ranges", "[ImgDetectionsFilter][CF-4][CF-5][ER-11]") {
    FilterSettings s;
    SECTION("reversed confidence") {
        s.minConfidence = .8f;
        s.maxConfidence = .5f;
    }
    SECTION("equal confidence") {
        s.minConfidence = .5f;
        s.maxConfidence = .5f;
    }
    SECTION("equal confidence at default maximum") {
        s.minConfidence = 1;
        s.maxConfidence = 1;
    }
    SECTION("reversed area") {
        s.minArea = 100;
        s.maxArea = 50;
    }
    SECTION("equal area") {
        s.minArea = 50;
        s.maxArea = 50;
    }
    SECTION("reversed width") {
        s.minWidth = 10;
        s.maxWidth = 5;
    }
    SECTION("reversed height") {
        s.minHeight = 10;
        s.maxHeight = 5;
    }
    FilterHarness h(s);
    REQUIRE_THROWS(h.pipeline.start());
}
TEST_CASE("ImgDetectionsFilter A-8: open ended confidence range", "[ImgDetectionsFilter][CF-4]") {
    FilterSettings s;
    s.minConfidence = .5f;
    FilterHarness h(s);
    REQUIRE_NOTHROW(h.pipeline.start());
}
TEST_CASE("ImgDetectionsFilter A-9: inputs cannot be linked after build", "[ImgDetectionsFilter][IN-2]") {
    FilterHarness h;
    h.accessUnlinked("new");
    h.pipeline.start();
    REQUIRE_THROWS(h.linkAccessedInput());
}
TEST_CASE("ImgDetectionsFilter A-10: all config options survive serialization", "[ImgDetectionsFilter][skill-E]") {
    const auto original = makeConfig(nondefaultSettings());
    const auto metadata = dai::utility::serialize(*original);
    dai::ImgDetectionsFilterConfig copy;
    dai::utility::deserialize(metadata, copy);
    REQUIRE(dai::utility::serialize(copy) == metadata);
    auto bytes = dai::StreamMessageParser::serializeMetadata(*original);
    streamPacketDesc_t packet{};
    packet.data = bytes.data();
    packet.length = bytes.size();
    packet.fd = -1;
    const auto parsed = std::dynamic_pointer_cast<dai::ImgDetectionsFilterConfig>(dai::StreamMessageParser::parseMessage(&packet));
    REQUIRE(parsed != nullptr);
    REQUIRE(parsed->getDatatype() == original->getDatatype());
    REQUIRE(dai::utility::serialize(*parsed) == metadata);
}

TEST_CASE("ImgDetectionsFilter B-1: default preserves all fields and input", "[ImgDetectionsFilter][CF-1][OUT-3][OUT-6][OUT-8][IN-5]") {
    runSharedCase("B-1");
}
TEST_CASE("ImgDetectionsFilter B-2: pure filter without transformation", "[ImgDetectionsFilter][ER-5]") {
    auto input = sharedCases("B-1").front().input;
    input->transformation.reset();
    const MessageSnapshot before(*input);
    FilterHarness h;
    h.pipeline.start();
    for(int i = 0; i < 2; ++i) {
        h.send(input);
        const auto out = h.receive();
        REQUIRE_FALSE(out->getTransformation().has_value());
        before.requireEqual(*out);
    }
    before.requireEqual(*input);
    REQUIRE(h.pipeline.isRunning());
}
TEST_CASE("ImgDetectionsFilter B-3: label selection", "[ImgDetectionsFilter][FL-1][FL-2][FL-3]") {
    runSharedCase("B-3");
}
TEST_CASE("ImgDetectionsFilter B-4: inclusive limits and standard form", "[ImgDetectionsFilter][CF-2][FL-4][GE-1][GE-2][ST-3]") {
    runSharedCase("B-4");
}
TEST_CASE("ImgDetectionsFilter B-5: one ended confidence ranges", "[ImgDetectionsFilter][FL-4]") {
    FilterSettings s;
    std::vector<ExpectedDetection> data, expected;
    SECTION("minimum only") {
        s.minConfidence = .5f;
        data = {{100, 100, 64, 64, .5f}, {200, 100, 64, 64, .49f}, {300, 100, 64, 64, 1}};
        expected = {data[0], data[2]};
    }
    SECTION("maximum only") {
        s.maxConfidence = .75f;
        data = {{100, 100, 64, 64, 0}, {200, 100, 64, 64, .75f}, {300, 100, 64, 64, .76f}};
        expected = {data[0], data[1]};
    }
    SECTION("default endpoints impose no limit") {
        data = {{100, 100, 64, 64, .3f}, {200, 100, 64, 64, 1.5f}};
        expected = data;
    }
    checkRound(s, {message(transformation(), data)}, expected);
}
TEST_CASE("ImgDetectionsFilter B-6: inclusive height in standard form", "[ImgDetectionsFilter][GE-2]") {
    FilterSettings s;
    s.minHeight = 64;
    s.maxHeight = 128;
    const std::vector<ExpectedDetection> data = {{256, 256, 100, 64},
                                                 {256, 256, 100, 63},
                                                 {256, 256, 100, 128},
                                                 {256, 256, 100, 129},
                                                 {256, 256, 50, 120, .9f, 0, 90},
                                                 {256, 256, 120, 100, .9f, 0, 90}};
    checkRound(s, {message(transformation(), data)}, {data[0], data[2], data[5]});
}
TEST_CASE("ImgDetectionsFilter B-7: output pixel geometry", "[ImgDetectionsFilter][GE-5][ST-3]") {
    FilterSettings s;
    s.minArea = 4000;
    std::vector<ExpectedDetection> expected;
    SECTION("pure") {}
    SECTION("remap") {
        s.reference = transformation();
        expected = {{128, 128, 64, 64}};
    }
    checkRound(s, {message(scaled(transformation(), .5f), {{64, 64, 32, 32}})}, expected, .01f, .01f);
}
TEST_CASE("ImgDetectionsFilter B-8: rotated ROI containment", "[ImgDetectionsFilter][GE-3][GE-4]") {
    runSharedCase("B-8");
}
TEST_CASE("ImgDetectionsFilter B-9: pixel marked boxes", "[ImgDetectionsFilter][IN-6]") {
    FilterSettings s;
    bool passes = false;
    SECTION("inclusive") {
        s.minArea = 4096;
        passes = true;
    }
    SECTION("too small") {
        s.minArea = 4097;
    }
    auto input = message(transformation(), {{64, 64, 64, 64}});
    input->detections[0].setBoundingBox({dai::Point2f(64, 64, false), dai::Size2f(64, 64, false), 0});
    checkRound(s, {input}, passes ? std::vector<ExpectedDetection>{{64, 64, 64, 64}} : std::vector<ExpectedDetection>{});
}
TEST_CASE("ImgDetectionsFilter B-10: legacy outer rectangle", "[ImgDetectionsFilter][OUT-4]") {
    FilterSettings s;
    bool passes = false;
    SECTION("wide enough") {
        s.minWidth = 100;
        passes = true;
    }
    SECTION("too narrow") {
        s.minWidth = 130;
    }
    auto input = message(transformation(), {});
    dai::ImgDetection d;
    d.confidence = .9f;
    d.xmin = .25f;
    d.ymin = .25f;
    d.xmax = .5f;
    d.ymax = .375f;
    input->detections.push_back(d);
    checkRound(s, {input}, passes ? std::vector<ExpectedDetection>{{192, 160, 128, 64}} : std::vector<ExpectedDetection>{});
}
TEST_CASE("ImgDetectionsFilter B-11: absent box passes default filter", "[ImgDetectionsFilter][ER-10]") {
    auto input = message(transformation(), {});
    dai::ImgDetection d;
    d.label = 1;
    d.confidence = .9f;
    input->detections.push_back(d);
    const MessageSnapshot before(*input);
    FilterHarness h;
    h.pipeline.start();
    for(int i = 0; i < 2; ++i) {
        h.send(input);
        const auto out = h.receive();
        before.requireEqual(*out);
        REQUIRE(out->detections.size() == 1);
        const auto& actual = out->detections[0];
        REQUIRE_FALSE(actual.boundingBox.has_value());
        REQUIRE(actual.xmin == 0);
        REQUIRE(actual.ymin == 0);
        REQUIRE(actual.xmax == 0);
        REQUIRE(actual.ymax == 0);
    }
    REQUIRE(h.pipeline.isRunning());
}

TEST_CASE("ImgDetectionsFilter D-1: maximum count and stable sort", "[ImgDetectionsFilter][MC-1][MC-2][SO-1][SO-2][TB-1]") {
    runSharedCase("D-1");
}
TEST_CASE("ImgDetectionsFilter D-2: maximum zero still emits a message", "[ImgDetectionsFilter][CF-6]") {
    FilterSettings s;
    s.maxDetections = 0;
    checkRound(s, {sharedCases("D-1").front().input}, {});
}
TEST_CASE("ImgDetectionsFilter D-3: lexicographic key order", "[ImgDetectionsFilter][OUT-1][SO-2]") {
    FilterSettings s;
    s.reference = transformation();
    bool sorted = false;
    SECTION("unsorted") {}
    SECTION("sorted") {
        s.sort = true;
        sorted = true;
    }
    const ExpectedDetection a0{100, 100, 64, 64, .6f}, a1{300, 100, 64, 64, .9f}, b0{100, 400, 64, 64, .7f, 1};
    FilterHarness h(s, {"cam2", "cam10"});
    h.pipeline.start();
    h.send(message(transformation(), {a0, a1}), "cam2");
    h.send(message(transformation(), {b0}), "cam10");
    requireOutput(*h.receive(), transformation(), sorted ? std::vector<ExpectedDetection>{a1, b0, a0} : std::vector<ExpectedDetection>{b0, a0, a1});
}

TEST_CASE("ImgDetectionsFilter E-1: mask reindexing", "[ImgDetectionsFilter][MK-1][MK-6][MK-13]") {
    runSharedCase("E-1");
}
TEST_CASE("ImgDetectionsFilter E-2: empty output mask and metadata", "[ImgDetectionsFilter][RD-5][MK-2][MK-4]") {
    runSharedCase("E-2");
}
TEST_CASE("ImgDetectionsFilter E-3: pure filter preserves mask resolution", "[ImgDetectionsFilter][MK-6]") {
    auto input = message(transformation(8, 4), maskDetections());
    input->setSegmentationMask(maskBytes("0 1 2 .\n0 . 2 2"), 4, 2);
    FilterSettings s;
    s.minConfidence = .5f;
    auto out = checkRound(s, {input}, {maskDetections()[0], maskDetections()[2]});
    requireMask(*out, 4, 2, "0 . 1 .\n0 . 1 1");
}
TEST_CASE("ImgDetectionsFilter E-4: invalid mask index", "[ImgDetectionsFilter][MK-16]") {
    const std::vector<ExpectedDetection> data = {{1, 2, 2, 4}, {6, 2, 2, 4}};
    auto input = message(transformation(8, 4), data);
    input->setSegmentationMask(maskBytes("0 7 1 ."), 4, 1);
    requireMask(*checkRound({}, {input}, data), 4, 1, "0 . 1 .");
}
TEST_CASE("ImgDetectionsFilter E-5: mask indices above 254", "[ImgDetectionsFilter][MK-17][ER-8]") {
    FilterSettings s;
    bool sorted = false;
    SECTION("input order") {}
    SECTION("descending confidence") {
        s.sort = true;
        sorted = true;
    }
    std::vector<ExpectedDetection> data;
    for(int i = 0; i < 300; ++i) data.push_back({256, 256, 64, 64, (i + 1) / 1000.f});
    auto input = message(transformation(), data);
    input->setSegmentationMask(maskBytes("0 1 254 ."), 4, 1);
    if(sorted) std::reverse(data.begin(), data.end());
    requireMask(*checkRound(s, {input}, data), 4, 1, sorted ? ". . 45 ." : "0 1 254 .");
}
TEST_CASE("ImgDetectionsFilter E-6: sorted mask indices", "[ImgDetectionsFilter][MK-1]") {
    FilterSettings s;
    s.sort = true;
    const auto data = maskDetections();
    requireMask(*checkRound(s, {maskMessage()}, {data[0], data[2], data[1]}), 8, 4, "0 0 . 2 2 . 1 1\n0 0 . 2 2 . 1 1\n0 0 . 2 2 . 1 1\n. . . 2 2 . . .");
}
TEST_CASE("ImgDetectionsFilter E-7: count removes mask pixels", "[ImgDetectionsFilter][MK-1][MK-11]") {
    FilterSettings s;
    s.maxDetections = 1;
    requireMask(*checkRound(s, {maskMessage()}, {maskDetections()[0]}), 8, 4, "0 0 . . . . . .\n0 0 . . . . . .\n0 0 . . . . . .\n. . . . . . . .");
}

TEST_CASE("ImgDetectionsFilter F-1: one output per round", "[ImgDetectionsFilter][RD-3]") {
    FilterHarness h;
    h.pipeline.start();
    for(int n = 1; n <= 3; ++n) {
        auto input = message();
        input->setSequenceNum(n);
        h.send(input);
    }
    for(int n = 1; n <= 3; ++n) {
        auto out = h.receive();
        REQUIRE(out->getSequenceNum() == n);
        requireOutput(*out, transformation(), {{100, 100, 64, 64, .9f, 1}});
    }
    h.requireNoOutput();
}
TEST_CASE("ImgDetectionsFilter F-2: wait for all keys", "[ImgDetectionsFilter][RD-2][RD-4]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s, {"a", "b"});
    h.pipeline.start();
    h.send(message(), "a");
    h.requireNoOutput();
    h.send(message(transformation(), {{300, 300, 64, 64, .9f, 1}}), "b");
    requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 1}, {300, 300, 64, 64, .9f, 1}});
}
TEST_CASE("ImgDetectionsFilter F-3: rounds follow arrival order not timestamps", "[ImgDetectionsFilter][RD-1][RD-2][OUT-1]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s, {"a", "b"});
    h.pipeline.start();
    for(int key = 0; key < 2; ++key) {
        for(int round = 0; round < 2; ++round) {
            auto input = message(transformation(), {{100.f + 200 * key, 100.f + 200 * round, 64, 64, .9f, static_cast<std::uint32_t>(11 + 10 * key + round)}});
            input->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(10 + 10 * key + round)));
            h.send(input, key == 0 ? "a" : "b");
        }
    }
    requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 11}, {300, 100, 64, 64, .9f, 21}});
    requireOutput(*h.receive(), transformation(), {{100, 300, 64, 64, .9f, 12}, {300, 300, 64, 64, .9f, 22}});
}
TEST_CASE("ImgDetectionsFilter F-4: empty round", "[ImgDetectionsFilter][RD-5]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(transformation(), {}), message(transformation(), {})}, {});
}
TEST_CASE("ImgDetectionsFilter F-5: Sync and MessageDemux integration", "[ImgDetectionsFilter][RD-1]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s, {"a", "b"}, false, false, true);
    h.pipeline.start();
    h.send(message(transformation(), {{100, 100, 64, 64, .9f, 11}}), "a");
    h.send(message(transformation(), {{300, 100, 64, 64, .9f, 21}}), "b");
    requireOutput(*h.receive(), transformation(), {{100, 100, 64, 64, .9f, 11}, {300, 100, 64, 64, .9f, 21}});
}

TEST_CASE("ImgDetectionsFilter G-1: reference latching between rounds",
          "[ImgDetectionsFilter][RF-1][RF-2][RF-4][RF-5][RF-6][RF-7][RF-9][RF-10][RT-5][ER-4][RM-10][RM-12][RM-13][MK-5][MK-8]") {
    FilterHarness h({}, {"cam"}, false, true);
    h.pipeline.start();
    for(int i = 0; i < 2; ++i) {
        auto input = message(transformation(8, 4), {{6, 2, 2, 2, .9f}, {1, 2, 2, 2, .8f}, {4, 2, 2, 2, .7f}});
        input->setSegmentationMask(maskBytes(". . . . . . . .\n1 1 . 2 2 0 0 .\n1 1 . 2 2 0 0 .\n. . . . . . . ."), 8, 4);
        h.send(input);
        h.requireNoOutput();
    }
    h.reference(transformation(8, 4));
    checkReferenceRound(h, false, true);
    h.reference(cropped(transformation(8, 4), 4, 0, 4, 4));
    FilterSettings s;
    s.reference = transformation(8, 4);
    h.configure(s);
    checkReferenceRound(h, true, true);
    h.reference(cropped(transformation(8, 4), 4, 0, 4, 4));
    checkReferenceRound(h, true, true);
    checkReferenceRound(h, true, true);
}
TEST_CASE("ImgDetectionsFilter G-2: config reference remains latched", "[ImgDetectionsFilter][RF-3][RT-3]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    FilterHarness h(s);
    h.pipeline.start();
    checkReferenceRound(h, false);
    s.reference = cropped(transformation(8, 4), 4, 0, 4, 4);
    h.configure(s);
    checkReferenceRound(h, true);
    s = {};
    s.minConfidence = 0;
    s.maxConfidence = 1;
    h.configure(s);
    checkReferenceRound(h, true);
}
TEST_CASE("ImgDetectionsFilter G-3: pure filtering switches to remap", "[ImgDetectionsFilter][RF-3]") {
    FilterHarness h;
    h.pipeline.start();
    checkReferenceRound(h, false);
    FilterSettings s;
    s.reference = cropped(transformation(8, 4), 4, 0, 4, 4);
    h.configure(s);
    checkReferenceRound(h, true);
    h.configure({});
    checkReferenceRound(h, true);
}
TEST_CASE("ImgDetectionsFilter G-4: first reference frame supersedes config", "[ImgDetectionsFilter][RF-4]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    FilterHarness h(s, {"cam"}, false, true);
    h.pipeline.start();
    checkReferenceRound(h, false);
    s.reference = cropped(transformation(8, 4), 4, 0, 4, 4);
    h.configure(s);
    checkReferenceRound(h, true);
    h.reference(transformation(8, 4));
    checkReferenceRound(h, false);
    h.configure(s);
    checkReferenceRound(h, false);
}
TEST_CASE("ImgDetectionsFilter G-5: invalid reference frame is ignored", "[ImgDetectionsFilter][RF-8][ER-7]") {
    FilterHarness h({}, {"cam"}, false, true);
    h.pipeline.start();
    h.reference(transformation(8, 4));
    checkReferenceRound(h, false);
    h.reference(dai::ImgTransformation());
    checkReferenceRound(h, false);
    checkReferenceRound(h, false);
    REQUIRE(h.pipeline.isRunning());
}
TEST_CASE("ImgDetectionsFilter G-6: dropped rounds consume every key", "[ImgDetectionsFilter][RF-10]") {
    FilterHarness h({}, {"a", "b"}, false, true);
    h.pipeline.start();
    h.send(message(transformation(), {{100, 100, 64, 64, .9f, 11}}), "a");
    h.send(message(transformation(), {{300, 100, 64, 64, .9f, 21}}), "b");
    h.requireNoOutput();
    h.reference(transformation());
    h.send(message(transformation(), {{100, 300, 64, 64, .9f, 12}}), "a");
    h.send(message(transformation(), {{300, 300, 64, 64, .9f, 22}}), "b");
    requireOutput(*h.receive(), transformation(), {{100, 300, 64, 64, .9f, 12}, {300, 300, 64, 64, .9f, 22}});
}
TEST_CASE("ImgDetectionsFilter G-7: pixel limits follow reference size", "[ImgDetectionsFilter][RT-7]") {
    FilterSettings s;
    s.reference = transformation();
    s.minArea = 4000;
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{256, 256, 64, 64}}));
    requireOutput(*h.receive(), transformation(), {{256, 256, 64, 64}});
    s.reference = scaled(transformation(), .5f);
    h.configure(s);
    h.send(message(transformation(), {{256, 256, 64, 64}}));
    requireOutput(*h.receive(), *s.reference, {});
}
TEST_CASE("ImgDetectionsFilter G-8: newest reference frame wins", "[ImgDetectionsFilter][RF-5][RF-7]") {
    FilterHarness h({}, {"cam"}, false, true);
    h.pipeline.start();
    h.reference(cropped(transformation(8, 4), 4, 0, 4, 4));
    h.reference(transformation(8, 4));
    checkReferenceRound(h, false);
}

TEST_CASE("ImgDetectionsFilter R-1: scale remap preserves input", "[ImgDetectionsFilter][RM-1][RM-2][RM-8][OUT-5][IN-5]") {
    runSharedCase("R-1");
}
TEST_CASE("ImgDetectionsFilter R-2a: outside border contact has no area", "[ImgDetectionsFilter][RM-12][ER-9]") {
    FilterSettings s;
    s.reference = cropped(transformation(8, 4), 4, 0, 4, 4);
    checkRound(s, {message(transformation(8, 4), {{3, 2, 2, 2, .9f, 5, 0, "obj"}})}, {});
}
TEST_CASE("ImgDetectionsFilter R-2b: partial outside box remains unclipped", "[ImgDetectionsFilter][RM-13]") {
    FilterSettings s;
    s.reference = cropped(transformation(8, 4), 4, 0, 4, 4);
    checkRound(s, {message(transformation(8, 4), {{3.5f, 2, 2, 2, .9f, 5, 0, "obj"}})}, {{-.5f, 2, 2, 2, .9f, 5, 0, "obj"}}, .01f, .01f);
}
TEST_CASE("ImgDetectionsFilter R-3: camera roll remap", "[ImgDetectionsFilter][RM-5][RM-6][RM-9][OUT-5][OUT-6]") {
    runSharedCase("R-3");
}
TEST_CASE("ImgDetectionsFilter R-4: thirty degree roll", "[ImgDetectionsFilter][RM-5][RM-6]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(
        s, {message(transformation(512, 512, rotation(30)), {{356, 256, 100, 40, .9f, 5, 0, "obj"}})}, {{342.60f, 306, 100, 40, .9f, 5, 30, "obj"}}, .05f, .1f);
}
TEST_CASE("ImgDetectionsFilter R-5: sixty degree roll standard form", "[ImgDetectionsFilter][RM-5]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(
        s, {message(transformation(512, 512, rotation(60)), {{256, 256, 100, 40, .9f, 5, 0, "obj"}})}, {{256, 256, 40, 100, .9f, 5, -30, "obj"}}, .05f, .1f);
}
TEST_CASE("ImgDetectionsFilter R-6a: equal transformation canonicalizes box", "[ImgDetectionsFilter][RM-10][RM-5]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(transformation(), {{256, 256, 100, 40, .9f, 5, 90, "obj"}})}, {{256, 256, 40, 100, .9f, 5, 0, "obj"}});
}
TEST_CASE("ImgDetectionsFilter R-6b: equal transformation preserves standard box", "[ImgDetectionsFilter][RM-10]") {
    FilterSettings s;
    s.reference = transformation();
    const std::vector<ExpectedDetection> data = {{200, 300, 40, 100, .9f, 5, 10, "obj"}};
    checkRound(s, {message(transformation(), data)}, data);
}
TEST_CASE("ImgDetectionsFilter R-7: remapped keypoints", "[ImgDetectionsFilter][RM-7][RM-8]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s,
               {message(transformation(512, 512, rotation(90)), {{300, 200, 100, 40, .9f, 5, 0, "obj", {{300, 200}, {350, 220, .8f}}}})},
               {{312, 300, 40, 100, .9f, 5, 0, "obj", {{312, 300}, {292, 350, .8f}}}},
               .05f,
               .1f);
}
TEST_CASE("ImgDetectionsFilter R-8: unprojectable keypoint retains its slot", "[ImgDetectionsFilter][RM-7]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(512, 512, rotation(60, true)), {{100, 256, 80, 80, .9f, 5, 0, "obj", {{100, 256}, {480, 256, .8f}}}}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const auto points = out->detections[0].getKeypoints();
    REQUIRE(points.size() == 2);
    REQUIRE_THAT(points[0].imageCoordinates.x * 512, Catch::Matchers::WithinAbs(395.82, .05));
    REQUIRE_THAT(points[0].imageCoordinates.y * 512, Catch::Matchers::WithinAbs(256, .05));
    REQUIRE_THAT(points[0].confidence, Catch::Matchers::WithinAbs(.9, 1e-6));
    REQUIRE(points[1].confidence == 0);
}
TEST_CASE("ImgDetectionsFilter R-9: behind perspective reference", "[ImgDetectionsFilter][RM-11][ER-9]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(transformation(512, 512, rotation(180, true)), {{256, 256, 100, 100, .9f, 5, 0, "obj"}})}, {});
}
TEST_CASE("ImgDetectionsFilter R-12: reference distortion", "[ImgDetectionsFilter][RM-3]") {
    const auto source = transformation();
    FilterSettings s;
    s.reference = transformation(512, 512, IDENTITY, dai::CameraModel::Perspective, {.3f, .05f, 0, 0, 0});
    FilterHarness h(s);
    const std::vector<ExpectedKeypoint> points = {{200, 200}, {300, 220}, {256, 300}};
    h.pipeline.start();
    h.send(message(source, {{256, 256, 100, 100, .9f, 5, 0, "obj", points}}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const auto actual = out->detections[0].getKeypoints();
    REQUIRE(actual.size() == points.size());
    for(std::size_t i = 0; i < points.size(); ++i) {
        const auto expected = source.remapPointTo(*s.reference, dai::Point2f(points[i].x, points[i].y, false));
        REQUIRE_THAT(actual[i].imageCoordinates.x * 512, Catch::Matchers::WithinAbs(expected.x, .05));
        REQUIRE_THAT(actual[i].imageCoordinates.y * 512, Catch::Matchers::WithinAbs(expected.y, .05));
        REQUIRE_THAT(actual[i].confidence, Catch::Matchers::WithinAbs(.9, 1e-6));
    }
}
TEST_CASE("ImgDetectionsFilter R-13a: standard form uses nonsquare pixel units", "[ImgDetectionsFilter][RM-5][RM-8][ST-3][OUT-5][GE-2]") {
    const auto t = transformation(512, 256);
    FilterSettings s;
    s.reference = t;
    const auto out = checkRound(s, {message(t, {{256, 128, 100, 40, .9f, 5, 90, "obj"}})}, {{256, 128, 40, 100, .9f, 5, 0, "obj"}});
    const auto& d = out->detections[0];
    REQUIRE(d.getBoundingBox().isNormalized());
    REQUIRE_THAT(d.xmin, Catch::Matchers::WithinAbs(.4609375, 1e-4 / 512));
    REQUIRE_THAT(d.xmax, Catch::Matchers::WithinAbs(.5390625, 1e-4 / 512));
    REQUIRE_THAT(d.ymin, Catch::Matchers::WithinAbs(.3046875, 1e-4 / 256));
    REQUIRE_THAT(d.ymax, Catch::Matchers::WithinAbs(.6953125, 1e-4 / 256));
}
TEST_CASE("ImgDetectionsFilter R-13b: nonsquare pure width filter", "[ImgDetectionsFilter][RM-5][RM-8][ST-3][OUT-5][GE-2]") {
    FilterSettings s;
    s.minWidth = 64;
    s.maxWidth = 128;
    const std::vector<ExpectedDetection> data = {{256, 128, 50, 120, .9f, 5, 90, "obj"}};
    const auto input = message(transformation(512, 256), data);
    const auto out = checkRound(s, {input}, data);
    MessageSnapshot(*input).requireEqual(*out);
}
TEST_CASE("ImgDetectionsFilter R-14a: translation is ignored", "[ImgDetectionsFilter][RM-2]") {
    auto source = transformation(512, 512, rotation(30));
    auto e = source.getExtrinsics();
    e.translation = {10, 0, 0};
    source.setExtrinsics(e);
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(source, {{356, 256, 100, 40, .9f, 5, 0, "obj"}})}, {{342.60f, 306, 100, 40, .9f, 5, 30, "obj"}}, .05f, .1f);
}
TEST_CASE("ImgDetectionsFilter R-14b: different reference intrinsics", "[ImgDetectionsFilter][RM-2]") {
    FilterSettings s;
    s.reference = transformation(512, 512, IDENTITY, dai::CameraModel::Perspective, {}, 512);
    checkRound(s, {message(transformation(), {{300, 200, 100, 40, .9f, 5, 0, "obj"}})}, {{344, 144, 200, 80, .9f, 5, 0, "obj"}}, .01f, .01f);
}
TEST_CASE("ImgDetectionsFilter R-14c: scale then camera rotation", "[ImgDetectionsFilter][RM-2]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s,
               {message(scaled(transformation(512, 512, rotation(90)), .5f), {{150, 100, 50, 20, .9f, 5, 0, "obj"}})},
               {{312, 300, 40, 100, .9f, 5, 0, "obj"}},
               .05f,
               .1f);
}
TEST_CASE("ImgDetectionsFilter R-14d: source distortion", "[ImgDetectionsFilter][RM-2]") {
    const auto source = transformation(512, 512, IDENTITY, dai::CameraModel::Perspective, {.3f, .05f, 0, 0, 0});
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s);
    const std::vector<ExpectedKeypoint> points = {{200, 200}, {300, 220}, {256, 300}};
    h.pipeline.start();
    h.send(message(source, {{256, 256, 100, 100, .9f, 5, 0, "obj", points}}));
    const auto out = h.receive();
    REQUIRE(out->detections.size() == 1);
    const auto actual = out->detections[0].getKeypoints();
    REQUIRE(actual.size() == points.size());
    for(std::size_t i = 0; i < points.size(); ++i) {
        const auto expected = source.remapPointTo(*s.reference, dai::Point2f(points[i].x, points[i].y, false));
        REQUIRE_THAT(actual[i].imageCoordinates.x * 512, Catch::Matchers::WithinAbs(expected.x, .05));
        REQUIRE_THAT(actual[i].imageCoordinates.y * 512, Catch::Matchers::WithinAbs(expected.y, .05));
        REQUIRE_THAT(actual[i].confidence, Catch::Matchers::WithinAbs(.9, 1e-6));
    }
}
TEST_CASE("ImgDetectionsFilter R-15: equal transformation still rejects outside", "[ImgDetectionsFilter][RM-10][RM-12]") {
    const std::vector<ExpectedDetection> data = {{600, 100, 64, 64, .9f, 5, 0, "obj"}, {100, 100, 64, 64, .8f, 5, 0, "obj"}};
    FilterSettings s;
    auto expected = data;
    SECTION("reference") {
        s.reference = transformation();
        expected = {data[1]};
    }
    SECTION("pure filter") {}
    checkRound(s, {message(transformation(), data)}, expected);
}
TEST_CASE("ImgDetectionsFilter R-17: a subset of corners is unprojectable", "[ImgDetectionsFilter][RM-11]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(transformation(512, 512, rotation(60, true)), {{400, 256, 80, 80, .9f, 5, 0, "obj"}})}, {});
}
TEST_CASE("ImgDetectionsFilter R-18: outside test uses rotated rectangle", "[ImgDetectionsFilter][RM-12][RM-13]") {
    FilterSettings s;
    s.reference = cropped(transformation(), 200, 200, 312, 312);
    checkRound(s,
               {message(transformation(), {{170, 170, 60, 60, .9f, 5, 30, "obj"}, {195, 195, 60, 60, .8f, 5, 30, "obj"}})},
               {{-5, -5, 60, 60, .8f, 5, 30, "obj"}},
               .01f,
               .01f);
}

TEST_CASE("ImgDetectionsFilter O-1: highest IoU duplicate from each key", "[ImgDetectionsFilter][OV-1][OV-4][OV-6][OV-7][OUT-2][OUT-5]") {
    for(const auto mode : {Mode::NMS, Mode::AVERAGE}) {
        for(const bool renamed : {false, true}) {
            DYNAMIC_SECTION("mode " << static_cast<int>(mode) << ", renamed=" << renamed) {
                FilterSettings s;
                s.reference = transformation();
                s.mode = mode;
                FilterHarness h(s, {renamed ? "z" : "a", "b"});
                const ExpectedDetection p{200, 200, 100, 100, .75f}, q1{210, 200, 100, 100, .25f}, q2{230, 200, 100, 100, .5f};
                auto leader = p;
                if(mode == Mode::AVERAGE) leader.x = 202.5f;
                h.pipeline.start();
                h.send(message(transformation(), {p}), renamed ? "z" : "a");
                h.send(message(transformation(), {q1, q2}), "b");
                const auto out = h.receive();
                requireOutput(*out,
                              transformation(),
                              renamed ? std::vector<ExpectedDetection>{q2, leader} : std::vector<ExpectedDetection>{leader, q2},
                              s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f,
                              s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f);
                if(mode == Mode::AVERAGE) {
                    const auto& d = out->detections[renamed ? 1 : 0];
                    REQUIRE_THAT(d.xmin, Catch::Matchers::WithinAbs(.2978515625, 1e-3 / 512));
                    REQUIRE_THAT(d.xmax, Catch::Matchers::WithinAbs(.4931640625, 1e-3 / 512));
                    REQUIRE_THAT(d.ymin, Catch::Matchers::WithinAbs(.29296875, 1e-3 / 512));
                    REQUIRE_THAT(d.ymax, Catch::Matchers::WithinAbs(.48828125, 1e-3 / 512));
                }
            }
        }
    }
}
TEST_CASE("ImgDetectionsFilter O-2: duplicates need same label and different keys", "[ImgDetectionsFilter][OV-1][OV-4]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a0{200, 200, 100, 100, .9f}, a1{205, 200, 100, 100, .85f}, b0{200, 200, 100, 100, .8f, 1}, b1{200, 200, 100, 100, .6f};
    checkRound(s, {message(transformation(), {a0, a1}), message(transformation(), {b0, b1})}, {a0, a1, b0});
}
TEST_CASE("ImgDetectionsFilter O-3: overlap modes", "[ImgDetectionsFilter][OV-1]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a{200, 200, 100, 100, .9f}, b{200, 200, 100, 100, .8f};
    std::vector<ExpectedDetection> expected{a};
    SECTION("off") {
        s.mode = Mode::OFF;
        expected.push_back(b);
    }
    SECTION("NMS") {
        s.mode = Mode::NMS;
    }
    SECTION("average") {
        s.mode = Mode::AVERAGE;
    }
    checkRound(s,
               {message(transformation(), {a}), message(transformation(), {b})},
               expected,
               s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f,
               s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f);
}
TEST_CASE("ImgDetectionsFilter O-4: strict IoU threshold", "[ImgDetectionsFilter][OV-1]") {
    FilterSettings s;
    s.reference = transformation();
    ExpectedDetection a{200, 200, 100, 100, .9f}, b{200, 200, 100, 50, .8f};
    bool both = true;
    SECTION("above IoU") {
        s.iou = .6f;
    }
    SECTION("below IoU") {
        s.iou = .4f;
        both = false;
    }
    SECTION("exactly equal IoU") {
        s.reference = transformation(8, 4);
        s.iou = .5f;
        a = {2, 2, 4, 4, .9f};
        b = {2, 2, 4, 2, .8f};
    }
    checkRound(s, {message(*s.reference, {a}), message(*s.reference, {b})}, both ? std::vector<ExpectedDetection>{a, b} : std::vector<ExpectedDetection>{a});
}
TEST_CASE("ImgDetectionsFilter O-5: rotated IoU", "[ImgDetectionsFilter][OV-2]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a{256, 256, 200, 20, .9f, 0, 30}, b{256, 256, 200, 20, .8f, 0, -30};
    checkRound(s, {message(transformation(), {a}), message(transformation(), {b})}, {a, b});
}
TEST_CASE("ImgDetectionsFilter O-6: one key never suppresses itself", "[ImgDetectionsFilter][OV-3]") {
    FilterSettings s;
    s.reference = transformation();
    SECTION("NMS") {
        s.mode = Mode::NMS;
    }
    SECTION("average") {
        s.mode = Mode::AVERAGE;
    }
    const std::vector<ExpectedDetection> data = {{200, 200, 100, 100, .9f}, {200, 200, 100, 100, .8f}};
    checkRound(s, {message(transformation(), data)}, data);
}
TEST_CASE("ImgDetectionsFilter O-7: three key grouping", "[ImgDetectionsFilter][OV-4][OV-7]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a{200, 200, 100, 100, .5f}, b{210, 200, 100, 80, .25f}, c{180, 200, 100, 100, .25f};
    ExpectedDetection expected = a;
    SECTION("NMS") {}
    SECTION("average") {
        s.mode = Mode::AVERAGE;
        expected = {197.5f, 200, 100, 95, .5f};
    }
    checkRound(s,
               {message(transformation(), {a}), message(transformation(), {b}), message(transformation(), {c})},
               {expected},
               s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f,
               s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f);
}
TEST_CASE("ImgDetectionsFilter O-8: equal IoU uses confidence", "[ImgDetectionsFilter][OV-5]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a1{200, 200, 100, 100, .6f, 0, 0, "a1"}, a2{200, 200, 100, 100, .8f, 0, 0, "a2"}, b{200, 200, 100, 100, .95f, 0, 0, "b1"};
    checkRound(s, {message(transformation(), {a1, a2}), message(transformation(), {b})}, {a1, b});
}
TEST_CASE("ImgDetectionsFilter O-9: equal IoU and confidence use order", "[ImgDetectionsFilter][TB-1]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a1{200, 200, 100, 100, .6f, 0, 0, "a1"}, a2{200, 200, 100, 100, .6f, 0, 0, "a2"}, b{200, 200, 100, 100, .95f, 0, 0, "b1"};
    checkRound(s, {message(transformation(), {a1, a2}), message(transformation(), {b})}, {a2, b});
}
TEST_CASE("ImgDetectionsFilter O-10: a duplicate cannot join a second group", "[ImgDetectionsFilter][OV-4]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a1{200, 200, 100, 100, .9f}, a2{280, 200, 100, 100, .7f}, b{240, 200, 100, 100, .8f};
    checkRound(s, {message(transformation(), {a1, a2}), message(transformation(), {b})}, {a1, a2});
}
TEST_CASE("ImgDetectionsFilter O-11: zero confidence uses equal weights", "[ImgDetectionsFilter][OV-7]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    checkRound(s,
               {message(transformation(), {{200, 200, 100, 100, 0}}), message(transformation(), {{220, 200, 100, 100, 0}})},
               {{210, 200, 100, 100, 0}},
               1e-3f,
               1e-3f);
}
TEST_CASE("ImgDetectionsFilter O-12: leader supplies name and keypoints", "[ImgDetectionsFilter][OV-7]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    ExpectedDetection a{200, 200, 100, 100, .75f, 0, 0, "person_a", {{204.8f, 204.8f}}};
    ExpectedDetection b{210, 200, 100, 100, .25f, 0, 0, "person_b", {{230.4f, 204.8f}}};
    ExpectedDetection expected = a;
    expected.x = 202.5f;
    SECTION("leader in first key") {}
    SECTION("O-12b: leader in second key") {
        a.confidence = .25f;
        b.confidence = .75f;
        expected = b;
        expected.x = 207.5f;
    }
    checkRound(s, {message(transformation(), {a}), message(transformation(), {b})}, {expected}, 1e-3f, 1e-3f);
}
TEST_CASE("ImgDetectionsFilter O-13: averaging across standard form boundary", "[ImgDetectionsFilter][OV-8]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a{256, 256, 40, 100, .75f, 0, 43}, b{256, 256, 100, 40, .25f, 0, -43};
    ExpectedDetection expected = a;
    SECTION("average") {
        s.mode = Mode::AVERAGE;
        expected.angle = 44;
    }
    SECTION("NMS") {
        s.mode = Mode::NMS;
    }
    checkRound(s,
               {message(transformation(), {a}), message(transformation(), {b})},
               {expected},
               s.mode == Mode::AVERAGE ? .1f : 1e-4f,
               s.mode == Mode::AVERAGE ? .1f : 1e-4f);
}
TEST_CASE("ImgDetectionsFilter O-14: no duplicates leaves boxes unchanged", "[ImgDetectionsFilter][OV-9]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    const ExpectedDetection a{100, 100, 100, 100, .9f}, b{400, 400, 100, 100, .8f};
    checkRound(s, {message(transformation(), {a}), message(transformation(), {b})}, {a, b});
}
TEST_CASE("ImgDetectionsFilter O-15: threshold above one", "[ImgDetectionsFilter][CF-6]") {
    FilterSettings s;
    s.reference = transformation();
    s.iou = 1.5f;
    const ExpectedDetection a{200, 200, 100, 100, .9f}, b{200, 200, 100, 100, .8f};
    checkRound(s, {message(transformation(), {a}), message(transformation(), {b})}, {a, b});
}
TEST_CASE("ImgDetectionsFilter O-16: groups do not chain over cameras", "[ImgDetectionsFilter][OV-4]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection a{200, 200, 100, 100, .75f}, b{240, 200, 100, 100, .25f}, c{280, 200, 100, 100, .125f};
    auto leader = a;
    SECTION("NMS") {}
    SECTION("average") {
        s.mode = Mode::AVERAGE;
        leader.x = 210;
    }
    checkRound(s,
               {message(transformation(), {a}), message(transformation(), {b}), message(transformation(), {c})},
               {leader, c},
               s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f,
               s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f);
}

TEST_CASE("ImgDetectionsFilter S-1: weighted overlap", "[ImgDetectionsFilter][ST-1]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    checkRound(s,
               {message(transformation(), {{200, 200, 100, 100, .75f}}), message(transformation(), {{240, 200, 100, 100, .25f}})},
               {{210, 200, 100, 100, .75f}},
               1e-3f,
               1e-3f);
}
TEST_CASE("ImgDetectionsFilter S-1b: confidence precedes averaging", "[ImgDetectionsFilter][ST-1]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    s.minConfidence = .5f;
    checkRound(
        s, {message(transformation(), {{200, 200, 100, 100, .75f}}), message(transformation(), {{240, 200, 100, 100, .25f}})}, {{200, 200, 100, 100, .75f}});
}
TEST_CASE("ImgDetectionsFilter S-2: area follows suppression", "[ImgDetectionsFilter][ST-1][GE-1]") {
    FilterSettings s;
    s.reference = transformation();
    s.maxArea = 9000;
    checkRound(s, {message(transformation(), {{200, 200, 100, 100, .9f}}), message(transformation(), {{205, 200, 90, 90, .8f}})}, {});
}
TEST_CASE("ImgDetectionsFilter S-2b: suppression without area limit", "[ImgDetectionsFilter][control]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(s, {message(transformation(), {{200, 200, 100, 100, .9f}}), message(transformation(), {{205, 200, 90, 90, .8f}})}, {{200, 200, 100, 100, .9f}});
}
TEST_CASE("ImgDetectionsFilter S-3: width follows averaging", "[ImgDetectionsFilter][ST-1][GE-2]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    s.minWidth = 90;
    checkRound(s, {message(transformation(), {{200, 200, 100, 100, .5f}}), message(transformation(), {{200, 200, 60, 100, .5f}})}, {});
}
TEST_CASE("ImgDetectionsFilter S-3b: NMS preserves leader width", "[ImgDetectionsFilter][control]") {
    FilterSettings s;
    s.reference = transformation();
    s.minWidth = 90;
    checkRound(s, {message(transformation(), {{200, 200, 100, 100, .5f}}), message(transformation(), {{200, 200, 60, 100, .5f}})}, {{200, 200, 100, 100, .5f}});
}
TEST_CASE("ImgDetectionsFilter S-4: maximum confidence precedes suppression", "[ImgDetectionsFilter][ST-1][FL-4]") {
    FilterSettings s;
    s.reference = transformation();
    s.maxConfidence = .85f;
    checkRound(
        s, {message(transformation(), {{200, 200, 100, 100, .9f}}), message(transformation(), {{200, 200, 100, 100, .8f}})}, {{200, 200, 100, 100, .8f}});
}
TEST_CASE("ImgDetectionsFilter S-5: ROI follows suppression", "[ImgDetectionsFilter][ST-1][GE-3]") {
    FilterSettings s;
    s.reference = transformation();
    s.roi = dai::Rect(100, 100, 200, 200, false);
    checkRound(s, {message(transformation(), {{150, 200, 100, 100, .6f}}), message(transformation(), {{140, 200, 100, 100, .8f}})}, {});
}
TEST_CASE("ImgDetectionsFilter S-5b: suppression without ROI", "[ImgDetectionsFilter][control]") {
    FilterSettings s;
    s.reference = transformation();
    checkRound(
        s, {message(transformation(), {{150, 200, 100, 100, .6f}}), message(transformation(), {{140, 200, 100, 100, .8f}})}, {{140, 200, 100, 100, .8f}});
}
TEST_CASE("ImgDetectionsFilter S-6: width precedes maximum count", "[ImgDetectionsFilter][ST-1][MC-1]") {
    FilterSettings s;
    s.reference = transformation();
    s.minWidth = 60;
    s.maxDetections = 1;
    checkRound(s, {message(transformation(), {{100, 100, 30, 64, .9f}, {200, 200, 64, 64, .8f}, {300, 300, 64, 64, .7f}})}, {{200, 200, 64, 64, .8f}});
}
TEST_CASE("ImgDetectionsFilter S-7a: remap creates duplicates", "[ImgDetectionsFilter][ST-1]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    checkRound(
        s, {message(*s.reference, {{6, 2, 2, 2, .9f}}), message(cropped(*s.reference, 4, 0, 4, 4), {{2, 2, 2, 2, .8f}})}, {{6, 2, 2, 2, .9f}}, .01f, .01f);
}
TEST_CASE("ImgDetectionsFilter S-7b: remap separates local equal boxes", "[ImgDetectionsFilter][ST-1]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    checkRound(s,
               {message(*s.reference, {{2, 2, 2, 2, .9f}}), message(cropped(*s.reference, 4, 0, 4, 4), {{2, 2, 2, 2, .8f}})},
               {{2, 2, 2, 2, .9f}, {6, 2, 2, 2, .8f}},
               .01f,
               .01f);
}
TEST_CASE("ImgDetectionsFilter S-7c: camera remap precedes averaging", "[ImgDetectionsFilter][ST-1][RM-6]") {
    FilterSettings s;
    s.reference = transformation();
    s.mode = Mode::AVERAGE;
    checkRound(s,
               {message(transformation(), {{312, 300, 40, 100, .75f}}), message(transformation(512, 512, rotation(90)), {{300, 200, 100, 40, .25f}})},
               {{312, 300, 40, 100, .75f}},
               .05f,
               .1f);
}
TEST_CASE("ImgDetectionsFilter S-8: suppression precedes maximum count", "[ImgDetectionsFilter][ST-1][MC-1]") {
    FilterSettings s;
    s.reference = transformation();
    s.maxDetections = 2;
    checkRound(s,
               {message(transformation(), {{200, 200, 100, 100, .9f}, {400, 400, 100, 100, .5f}}), message(transformation(), {{200, 200, 100, 100, .8f}})},
               {{200, 200, 100, 100, .9f}, {400, 400, 100, 100, .5f}});
}

TEST_CASE("ImgDetectionsFilter M-1: masks follow duplicate mode", "[ImgDetectionsFilter][OV-6][OV-7][MK-14][MK-15][IN-5]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    bool average = false;
    SECTION("NMS") {}
    SECTION("average") {
        s.mode = Mode::AVERAGE;
        average = true;
    }
    auto a = message(*s.reference, {{3, 2, 4, 4, .75f}}), b = message(*s.reference, {{4, 2, 4, 4, .25f}});
    const std::string maskA = ". . 0 0 . . . .\n. 0 0 0 0 . . .\n. 0 0 0 0 . . .\n. 0 . . 0 . . .";
    a->setSegmentationMask(maskBytes(maskA), 8, 4);
    b->setSegmentationMask(maskBytes(". . . 0 0 . . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 . . 0 . ."), 8, 4);
    const MessageSnapshot beforeA(*a), beforeB(*b);
    const auto out =
        checkRound(s, {a, b}, {{average ? 3.25f : 3.f, 2, 4, 4, .75f}}, s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f, s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f);
    requireMask(*out, 8, 4, average ? ". . 0 0 0 . . .\n. 0 0 0 0 0 . .\n. 0 0 0 0 0 . .\n. 0 0 . 0 0 . ." : maskA);
    beforeA.requireEqual(*a);
    beforeB.requireEqual(*b);
}
TEST_CASE("ImgDetectionsFilter M-2: mask conflict uses confidence then order", "[ImgDetectionsFilter][MK-12][MK-1][TB-1]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    ExpectedDetection a{2, 2, 4, 4, .5f}, b{4, 2, 4, 4, .75f, 1};
    std::string row;
    bool sorted = false;
    SECTION("higher confidence") {
        row = "0 0 1 1 1 1 . .";
    }
    SECTION("equal confidence") {
        b.confidence = .5f;
        row = "0 0 0 0 1 1 . .";
    }
    SECTION("sorted") {
        s.sort = true;
        sorted = true;
        row = "1 1 0 0 0 0 . .";
    }
    auto inA = message(*s.reference, {a}), inB = message(*s.reference, {b});
    inA->setSegmentationMask(maskBytes("0 0 0 0 . . . .\n0 0 0 0 . . . .\n0 0 0 0 . . . .\n0 0 0 0 . . . ."), 8, 4);
    inB->setSegmentationMask(maskBytes(". . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . ."), 8, 4);
    const auto out = checkRound(s, {inA, inB}, sorted ? std::vector<ExpectedDetection>{b, a} : std::vector<ExpectedDetection>{a, b});
    requireMask(*out, 8, 4, row + "\n" + row + "\n" + row + "\n" + row);
}
TEST_CASE("ImgDetectionsFilter M-3: maskless inputs never paint boxes", "[ImgDetectionsFilter][MK-2][MK-3][MK-10][MK-14][MK-15]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    ExpectedDetection a{2, 2, 4, 4, .9f}, b{6, 2, 4, 4, .6f, 1};
    std::vector<ExpectedDetection> expected{a, b};
    std::string inputRow = ". . . . 0 0 0 0", outputRow = ". . . . 1 1 1 1";
    SECTION("M-3a: different labels") {}
    SECTION("M-3b: NMS drops duplicate mask") {
        a.x = b.x = 4;
        b.label = 0;
        expected = {a};
        inputRow = ". . 0 0 0 0 . .";
        outputRow = ". . . . . . . .";
    }
    SECTION("M-3b: average inherits duplicate mask") {
        s.mode = Mode::AVERAGE;
        a.x = b.x = 4;
        b.label = 0;
        expected = {a};
        inputRow = outputRow = ". . 0 0 0 0 . .";
    }
    auto inB = message(*s.reference, {b});
    inB->setSegmentationMask(maskBytes(inputRow + "\n" + inputRow + "\n" + inputRow + "\n" + inputRow), 8, 4);
    const auto out =
        checkRound(s, {message(*s.reference, {a}), inB}, expected, s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f, s.mode == Mode::AVERAGE ? 1e-3f : 1e-4f);
    requireMask(*out, 8, 4, outputRow + "\n" + outputRow + "\n" + outputRow + "\n" + outputRow);
}
TEST_CASE("ImgDetectionsFilter M-4: small mask scales to reference", "[ImgDetectionsFilter][MK-5][MK-7][MK-8]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    const std::vector<ExpectedDetection> data = {{1, 1, 2, 2, .9f}, {6, 2, 4, 4, .8f}};
    auto input = message(*s.reference, data);
    input->setSegmentationMask(maskBytes("0 . 1 1\n. . 1 ."), 4, 2);
    requireMask(*checkRound(s, {input}, data), 8, 4, "0 0 . . 1 1 1 1\n0 0 . . 1 1 1 1\n. . . . 1 1 . .\n. . . . 1 1 . .");
}
TEST_CASE("ImgDetectionsFilter M-5: final survivors own pixels", "[ImgDetectionsFilter][MK-11][MK-12][MK-13]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    const ExpectedDetection a{2, 2, 4, 4, .9f}, b{4, 2, 4, 4, .7f, 1};
    std::vector<ExpectedDetection> expected{a, b};
    std::string row;
    SECTION("default") {
        row = "0 0 0 0 1 1 . .";
    }
    SECTION("ROI removes initial pixel winner") {
        s.roi = dai::Rect(1, -1, 7, 6, false);
        expected = {b};
        row = ". . 0 0 0 0 . .";
    }
    auto inA = message(*s.reference, {a}), inB = message(*s.reference, {b});
    inA->setSegmentationMask(maskBytes("0 0 0 0 . . . .\n0 0 0 0 . . . .\n0 0 0 0 . . . .\n0 0 0 0 . . . ."), 8, 4);
    inB->setSegmentationMask(maskBytes(". . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . ."), 8, 4);
    requireMask(*checkRound(s, {inA, inB}, expected), 8, 4, row + "\n" + row + "\n" + row + "\n" + row);
}
TEST_CASE("ImgDetectionsFilter M-6: cropped camera mask remap", "[ImgDetectionsFilter][MK-5][MK-8][MK-9]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    auto a = message(*s.reference, {{1, 2, 2, 2, .9f}}), b = message(cropped(*s.reference, 4, 0, 4, 4), {{2, 2, 2, 2, .8f, 1}});
    a->setSegmentationMask(maskBytes(". . . . . . . .\n0 0 . . . . . .\n0 0 . . . . . .\n. . . . . . . ."), 8, 4);
    b->setSegmentationMask(maskBytes(". . . .\n. 0 0 .\n. 0 0 .\n. . . ."), 4, 4);
    requireMask(*checkRound(s, {a, b}, {{1, 2, 2, 2, .9f}, {6, 2, 2, 2, .8f, 1}}, .01f, .01f),
                8,
                4,
                ". . . . . . . .\n0 0 . . . 1 1 .\n0 0 . . . 1 1 .\n. . . . . . . .");
}
TEST_CASE("ImgDetectionsFilter M-7: empty survivors retain mask presence", "[ImgDetectionsFilter][MK-4]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    s.minConfidence = .95f;
    auto b = message(*s.reference, {{6, 2, 4, 4, .6f, 1}});
    b->setSegmentationMask(maskBytes(". . . . 0 0 0 0\n. . . . 0 0 0 0\n. . . . 0 0 0 0\n. . . . 0 0 0 0"), 8, 4);
    requireMask(
        *checkRound(s, {message(*s.reference, {{2, 2, 4, 4, .9f}}), b}, {}), 8, 4, ". . . . . . . .\n. . . . . . . .\n. . . . . . . .\n. . . . . . . .");
}
TEST_CASE("ImgDetectionsFilter M-8: noninteger mask scale samples pixel centers", "[ImgDetectionsFilter][MK-7][MK-8]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    const std::vector<ExpectedDetection> data = {{1, 2, 2, 2, .9f}, {4, 2, 2, 2, .8f}, {7, 2, 2, 2, .7f}};
    auto input = message(*s.reference, data);
    input->setSegmentationMask(maskBytes("0 1 2"), 3, 1);
    requireMask(*checkRound(s, {input}, data), 8, 4, "0 0 0 1 1 2 2 2\n0 0 0 1 1 2 2 2\n0 0 0 1 1 2 2 2\n0 0 0 1 1 2 2 2");
}
TEST_CASE("ImgDetectionsFilter M-9: rotated camera mask", "[ImgDetectionsFilter][MK-8]") {
    FilterSettings s;
    s.reference = transformation(4, 4);
    auto input = message(transformation(4, 4, rotation(90)), {{1, 1, 2, 2, .9f}, {2, 3, 4, 2, .8f}});
    input->setSegmentationMask(maskBytes("0 0 . .\n0 0 . .\n1 1 1 1\n. 1 1 ."), 4, 4);
    requireMask(*checkRound(s, {input}, {{3, 1, 2, 2, .9f}, {1, 2, 2, 4, .8f}}, .05f, .1f), 4, 4, ". 1 0 0\n1 1 0 0\n1 1 . .\n. 1 . .");
}
TEST_CASE("ImgDetectionsFilter M-10: unrepresentable index cannot win a pixel", "[ImgDetectionsFilter][MK-17][MK-12][control]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    std::size_t count = 255;
    std::string row;
    SECTION("index 255") {
        row = "0 0 0 0 . . . .";
    }
    SECTION("control index 254") {
        count = 254;
        row = "0 0 254 254 254 254 . .";
    }
    std::vector<ExpectedDetection> expected(count, {2, 2, 4, 4, .5f});
    auto a = message(*s.reference, expected), b = message(*s.reference, {{4, 2, 4, 4, .9f, 1}});
    expected.push_back({4, 2, 4, 4, .9f, 1});
    a->setSegmentationMask(maskBytes("0 0 0 0 . . . .\n0 0 0 0 . . . .\n0 0 0 0 . . . .\n0 0 0 0 . . . ."), 8, 4);
    b->setSegmentationMask(maskBytes(". . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . ."), 8, 4);
    requireMask(*checkRound(s, {a, b}, expected), 8, 4, row + "\n" + row + "\n" + row + "\n" + row);
}

TEST_CASE("ImgDetectionsFilter T-1: metadata from newest host timestamp", "[ImgDetectionsFilter][OUT-7][OUT-8]") {
    FilterSettings s;
    s.reference = transformation();
    const ExpectedDetection d{100, 100, 64, 64, .6f};
    auto a = message(transformation(), {d}), b = message(transformation(), {});
    const auto t0 = std::chrono::steady_clock::time_point(std::chrono::seconds(10));
    const auto s0 = std::chrono::system_clock::time_point(std::chrono::seconds(20));
    a->setSequenceNum(10);
    a->setTimestamp(t0 + std::chrono::milliseconds(1000));
    a->setTimestampDevice(t0 + std::chrono::milliseconds(5000));
    a->setTimestampSystem(s0 + std::chrono::seconds(1));
    b->setSequenceNum(57);
    b->setTimestamp(t0 + std::chrono::milliseconds(1003));
    b->setTimestampDevice(t0 + std::chrono::milliseconds(2000));
    b->setTimestampSystem(s0 + std::chrono::seconds(2));
    std::vector<std::shared_ptr<dai::ImgDetections>> inputs{a, b};
    auto source = b;
    SECTION("newest message can be empty") {}
    SECTION("equal host timestamps choose first key") {
        b->setTimestamp(a->getTimestamp());
        source = a;
    }
    SECTION("one key") {
        inputs = {a};
        source = a;
    }
    requireMetadata(*checkRound(s, inputs, {d}), *source);
}

TEST_CASE("ImgDetectionsFilter U-1: runtime config replaces all filters", "[ImgDetectionsFilter][RT-1]") {
    const ExpectedDetection x{100, 100, 64, 64, .9f, 1}, y{300, 300, 64, 64, .3f, 2};
    FilterSettings s;
    s.minConfidence = .5f;
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {x, y}));
    requireOutput(*h.receive(), transformation(), {x});
    s = {};
    s.reject = std::vector<std::uint32_t>{1};
    h.configure(s);
    h.send(message(transformation(), {x, y}));
    requireOutput(*h.receive(), transformation(), {y});
}
TEST_CASE("ImgDetectionsFilter U-2: last queued runtime config wins", "[ImgDetectionsFilter][RT-2]") {
    const std::vector<ExpectedDetection> data = {{100, 100, 64, 64, .9f, 1}, {300, 300, 64, 64, .3f, 2}};
    FilterHarness h;
    h.pipeline.start();
    FilterSettings s;
    s.minConfidence = .95f;
    h.configure(s);
    s.minConfidence = .2f;
    h.configure(s);
    h.send(message(transformation(), data));
    requireOutput(*h.receive(), transformation(), data);
}
TEST_CASE("ImgDetectionsFilter U-3: transformation changes without restart", "[ImgDetectionsFilter][RT-4][RT-6]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {{128, 128, 64, 64}}));
    const auto first = h.receive();
    requireOutput(*first, transformation(), {{128, 128, 64, 64}}, .01f, .01f);
    h.send(message(cropped(transformation(), 256, 0, 256, 512), {{128, 128, 64, 64}}));
    requireOutput(*h.receive(), transformation(), {{384, 128, 64, 64}}, .01f, .01f);
    h.send(message(transformation(), {{128, 128, 64, 64}}));
    MessageSnapshot(*first).requireEqual(*h.receive());
}
TEST_CASE("ImgDetectionsFilter U-4: identical rounds are deterministic", "[ImgDetectionsFilter][ST-2]") {
    FilterSettings s;
    s.reference = transformation(8, 4);
    s.mode = Mode::AVERAGE;
    FilterHarness h(s, {"a", "b"});
    auto a = message(*s.reference, {{3, 2, 4, 4, .75f}}), b = message(*s.reference, {{4, 2, 4, 4, .25f}});
    a->setSegmentationMask(maskBytes(". . 0 0 . . . .\n. 0 0 0 0 . . .\n. 0 0 0 0 . . .\n. 0 . . 0 . . ."), 8, 4);
    b->setSegmentationMask(maskBytes(". . . 0 0 . . .\n. . 0 0 0 0 . .\n. . 0 0 0 0 . .\n. . 0 . . 0 . ."), 8, 4);
    h.pipeline.start();
    h.send(a, "a");
    h.send(b, "b");
    const auto first = h.receive();
    requireOutput(*first, *s.reference, {{3.25f, 2, 4, 4, .75f}}, 1e-3f, 1e-3f);
    requireMask(*first, 8, 4, ". . 0 0 0 . . .\n. 0 0 0 0 0 . .\n. 0 0 0 0 0 . .\n. 0 0 . 0 0 . .");
    h.send(a, "a");
    h.send(b, "b");
    MessageSnapshot(*first).requireEqual(*h.receive());
}
TEST_CASE("ImgDetectionsFilter U-5: invalid runtime config is ignored", "[ImgDetectionsFilter][CF-5]") {
    const ExpectedDetection x{100, 100, 64, 64, .9f, 1}, y{300, 300, 64, 64, .3f, 2};
    FilterSettings s;
    s.minConfidence = .5f;
    FilterHarness h(s);
    h.pipeline.start();
    h.send(message(transformation(), {x, y}));
    requireOutput(*h.receive(), transformation(), {x});
    s.minConfidence = .8f;
    s.maxConfidence = .5f;
    h.configure(s);
    for(int i = 0; i < 2; ++i) {
        h.send(message(transformation(), {x, y}));
        requireOutput(*h.receive(), transformation(), {x});
    }
    REQUIRE(h.pipeline.isRunning());
    s = {};
    s.minConfidence = .2f;
    h.configure(s);
    h.send(message(transformation(), {x, y}));
    requireOutput(*h.receive(), transformation(), {x, y});
}
