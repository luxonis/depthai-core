#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <cmath>
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

#include "pipeline/utilities/ImgDetectionsFilter/ImgDetectionsFilterImpl.hpp"

using Catch::Matchers::WithinAbs;
using Config = dai::ImgDetectionsFilterConfig;
using Mode = Config::OverlapMode;

namespace {
std::shared_ptr<dai::ImgDetections> message(std::size_t width = 512, std::size_t height = 512) {
    auto result = std::make_shared<dai::ImgDetections>();
    result->setTransformation(dai::ImgTransformation(width, height));
    return result;
}
dai::ImgDetection detection(float x,
                            float y,
                            float width,
                            float height,
                            float confidence,
                            std::uint32_t label = 0,
                            float angle = 0,
                            std::size_t imageWidth = 512,
                            std::size_t imageHeight = 512) {
    return dai::ImgDetection(
        dai::RotatedRect(dai::Point2f(x, y, false), dai::Size2f(width, height, false), angle).normalize(imageWidth, imageHeight), confidence, label);
}
std::shared_ptr<dai::ImgDetections> process(const std::vector<std::shared_ptr<dai::ImgDetections>>& messages, const Config& config = {}, bool remap = false) {
    return dai::impl::filterDetectionRound(messages, config, remap ? messages.front()->getTransformation() : config.reference);
}
struct RunningFilter {
    dai::Pipeline pipeline{false};
    std::shared_ptr<dai::node::ImgDetectionsFilter> node = pipeline.create<dai::node::ImgDetectionsFilter>();
    std::shared_ptr<dai::MessageQueue> output = node->out.createOutputQueue();
    std::vector<std::shared_ptr<dai::InputQueue>> inputs;
    explicit RunningFilter(const std::vector<std::string>& keys = {"cam"}) {
        for(const auto& key : keys) inputs.push_back(node->inputs[key].createInputQueue(4, true));
    }
    std::shared_ptr<dai::ImgDetections> receive() {
        bool timedOut = false;
        auto result = output->get<dai::ImgDetections>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(result);
        return result;
    }
    ~RunningFilter() {
        pipeline.stop();
    }
};
}  // namespace

TEST_CASE("S1/S2/S13/S14: pure filtering, labels, count and mask reindexing", "[ImgDetectionsFilter]") {
    auto input = message(8, 4);
    input->detections = {detection(1, 2, 2, 4, .9f, 1, 0, 8, 4), detection(4, 2, 2, 4, .25f, 2, 0, 8, 4), detection(7, 2, 2, 4, .75f, 1, 0, 8, 4)};
    const std::vector<std::uint8_t> mask = {0, 0, 255, 1, 1, 255, 2, 2, 0,   0,   255, 1, 1, 255, 2,   2,
                                            0, 0, 255, 1, 1, 255, 2, 2, 255, 255, 255, 1, 1, 255, 255, 255};
    input->setSegmentationMask(mask, 8, 4);
    REQUIRE(dai::utility::serialize(*process({input})) == dai::utility::serialize(*input));
    REQUIRE(*process({input})->getMaskData() == mask);
    Config config;
    config.setConfidenceRange(.5f);
    auto result = process({input}, config);
    REQUIRE(result->detections.size() == 2);
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>{0, 0, 255, 255, 255, 255, 1, 1, 0,   0,   255, 255, 255, 255, 1,   1,
                                                                0, 0, 255, 255, 255, 255, 1, 1, 255, 255, 255, 255, 255, 255, 255, 255});
    REQUIRE(*input->getMaskData() == mask);  // IN-5
    config.labelsToKeep = std::vector<std::uint32_t>{1, 2};
    config.labelsToReject = std::vector<std::uint32_t>{1};
    REQUIRE(process({input}, config)->detections.empty());
    config = {};
    config.labelsToKeep = std::vector<std::uint32_t>{};
    result = process({input}, config);
    REQUIRE(result->detections.empty());
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>(32, 255));
    config = {};
    config.maxDetections = 0;
    REQUIRE(process({input}, config)->detections.empty());

    input = message();
    for(const auto confidence : {.5f, .75f, .875f, .75f}) input->detections.push_back(detection(200, 200, 64, 64, confidence));
    config = {};
    config.maxDetections = 2;
    result = process({input}, config);
    REQUIRE(result->detections.size() == 2);
    REQUIRE(result->detections[0].confidence == .75f);
    REQUIRE(result->detections[1].confidence == .875f);
    config.sortByConfidence = true;
    REQUIRE(process({input}, config)->detections[0].confidence == .875f);
}

TEST_CASE("S3/S10: inclusive output pixel geometry and rotated ROI", "[ImgDetectionsFilter]") {
    auto input = message();
    input->detections = {detection(64, 64, 64, 64, .5f),
                         detection(192, 64, 128, 64, .75f),
                         detection(320, 64, 64, 64, .4375f),
                         detection(448, 64, 64, 64, .8125f),
                         detection(64, 192, 64, 63, .625f),
                         detection(192, 192, 63, 128, .625f),
                         detection(320, 192, 129, 32, .625f),
                         detection(448, 192, 50, 120, .625f, 0, 90)};
    Config config;
    config.setConfidenceRange(.5f, .75f).setSizeRange(4096, 8192).setWidthRange(64, 128);
    auto result = process({input}, config);
    REQUIRE(result->detections.size() == 3);
    REQUIRE(result->detections[2].boundingBox->angle == 90);  // Pure filters preserve stored geometry.
    config = {};
    config.regionOfInterest = dai::Rect(100, 100, 200, 200, false);
    input->detections = {detection(150, 150, 100, 100, .9f),
                         detection(250, 250, 100, 100, .9f),
                         detection(260, 200, 100, 100, .9f),
                         detection(400, 400, 50, 50, .9f),
                         detection(200, 200, 300, 300, .9f),
                         detection(200, 200, 260, 20, .9f, 0, 45),
                         detection(200, 200, 260, 20, .9f)};
    REQUIRE(process({input}, config)->detections.size() == 3);
    input = message(256, 256);
    input->detections = {detection(64, 64, 32, 32, .9f, 0, 0, 256, 256)};
    config = {};
    config.minArea = 4000;
    REQUIRE(process({input}, config)->detections.empty());
    auto reference = *input->getTransformation();
    reference.addScale(2, 2).setSize(512, 512);
    config.reference = reference;
    REQUIRE(process({input}, config)->detections.size() == 1);
}

TEST_CASE("S5/S7/S8/S9: one highest-IoU duplicate per other key and processing order", "[ImgDetectionsFilter]") {
    auto a = message(), b = message();
    a->detections = {detection(200, 200, 100, 100, .75f)};
    b->detections = {detection(210, 200, 100, 100, .25f), detection(230, 200, 100, 100, .5f)};
    auto result = process({a, b}, {}, true);
    REQUIRE(result->detections.size() == 2);
    REQUIRE(result->detections[1].confidence == .5f);
    Config config;
    config.overlapMode = Mode::AVERAGE;
    result = process({a, b}, config, true);
    REQUIRE_THAT(result->detections[0].boundingBox->center.x * 512, WithinAbs(202.5, 1e-4));
    REQUIRE(result->detections[1].confidence == .5f);
    result = process({b, a}, config, true);
    REQUIRE(result->detections[0].confidence == .5f);
    REQUIRE(result->detections[1].confidence == .75f);
    a->detections = {detection(200, 200, 100, 100, .9f), detection(205, 200, 100, 100, .85f)};
    b->detections = {detection(200, 200, 100, 100, .8f, 1), detection(200, 200, 100, 100, .6f)};
    REQUIRE(process({a, b}, {}, true)->detections.size() == 3);
    a->detections = {detection(200, 200, 100, 100, .75f)};
    b->detections = {detection(240, 200, 100, 100, .25f)};
    REQUIRE_THAT(process({a, b}, config, true)->detections[0].boundingBox->center.x * 512, WithinAbs(210, 1e-4));
    config.minConfidence = .5f;
    REQUIRE(process({a, b}, config, true)->detections[0].boundingBox->center.x == a->detections[0].boundingBox->center.x);
    a->detections = {detection(200, 200, 100, 100, .9f)};
    b->detections = {detection(205, 200, 90, 90, .8f)};
    config = {};
    config.maxArea = 9000;
    REQUIRE(process({a, b}, config, true)->detections.empty());
    config = {};
    config.overlapMode = Mode::OFF;
    REQUIRE(process({a, b}, config, true)->detections.size() == 2);
    config = {};
    config.overlapIouThreshold = 1;
    b->detections = a->detections;
    REQUIRE(process({a, b}, config, true)->detections.size() == 2);  // Strict > comparison.
}

TEST_CASE("S6/S11/S12: final mask ownership, NMS and averaged unions", "[ImgDetectionsFilter]") {
    auto a = message(8, 4), b = message(8, 4);
    a->detections = {detection(3, 2, 4, 4, .75f, 0, 0, 8, 4)};
    b->detections = {detection(4, 2, 4, 4, .25f, 0, 0, 8, 4)};
    const std::vector<std::uint8_t> am = {255, 255, 0, 0, 255, 255, 255, 255, 255, 0, 0,   0,   0, 255, 255, 255,
                                          255, 0,   0, 0, 0,   255, 255, 255, 255, 0, 255, 255, 0, 255, 255, 255};
    const std::vector<std::uint8_t> bm = {255, 255, 255, 0, 0, 255, 255, 255, 255, 255, 0, 0,   0,   0, 255, 255,
                                          255, 255, 0,   0, 0, 0,   255, 255, 255, 255, 0, 255, 255, 0, 255, 255};
    a->setSegmentationMask(am, 8, 4);
    b->setSegmentationMask(bm, 8, 4);
    REQUIRE(*process({a, b}, {}, true)->getMaskData() == am);
    Config config;
    config.overlapMode = Mode::AVERAGE;
    auto result = process({a, b}, config, true);
    REQUIRE_THAT(result->detections[0].boundingBox->center.x, WithinAbs(.40625, 1e-6));
    auto united = am;
    for(std::size_t i = 0; i < am.size(); ++i)
        if(bm[i] != 255) united[i] = 0;
    REQUIRE(*result->getMaskData() == united);
    a = message(8, 4);
    a->detections = {detection(4, 2, 4, 4, .9f, 0, 0, 8, 4)};
    REQUIRE(*process({a, b}, {}, true)->getMaskData() == std::vector<std::uint8_t>(32, 255));
    REQUIRE(*process({a, b}, config, true)->getMaskData() == bm);
    a->detections[0].label = 1;
    a->setSegmentationMask(std::vector<std::uint8_t>(32, 0), 8, 4);
    result = process({a, b}, {}, true);
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>(32, 0));
    config = {};
    config.maxArea = 15;  // Remove both by geometry => no stale mask values.
    REQUIRE(*process({a, b}, config, true)->getMaskData() == std::vector<std::uint8_t>(32, 255));
    b->detections[0].boundingBox->size.width = .25f;
    result = process({a, b}, config, true);  // Lower confidence survivor inherits pixels once higher one is filtered.
    REQUIRE(result->detections.size() == 1);
    REQUIRE(*result->getMaskData() == bm);
}

TEST_CASE("S15: newest metadata, equal timestamps choose earlier key", "[ImgDetectionsFilter]") {
    auto a = message(), b = message();
    const auto time = std::chrono::steady_clock::now();
    a->setTimestamp(time);
    b->setTimestamp(time + std::chrono::milliseconds(3));
    a->setSequenceNum(10);
    b->setSequenceNum(57);
    b->setTimestampDevice(time + std::chrono::seconds(2));
    auto result = process({a, b}, {}, true);
    REQUIRE(result->getSequenceNum() == 57);
    REQUIRE(result->getTimestampDevice() == b->getTimestampDevice());
    b->setTimestamp(time);
    REQUIRE(process({a, b}, {}, true)->getSequenceNum() == 10);
}

TEST_CASE("Remapping canonical angles, pixels, crop bounds, keypoints and mask resolution", "[ImgDetectionsFilter]") {
    auto input = message(8, 4);
    input->detections = {detection(6, 2, 2, 2, .9f, 0, 0, 8, 4), detection(1, 2, 2, 2, .8f, 0, 0, 8, 4), detection(4, 2, 2, 2, .7f, 0, 0, 8, 4)};
    input->setSegmentationMask(
        {255, 255, 255, 255, 255, 255, 255, 255, 1, 1, 255, 2, 2, 0, 0, 255, 1, 1, 255, 2, 2, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255}, 8, 4);
    Config config;
    config.reference = *input->getTransformation();
    config.reference->addCrop(4, 0, 4, 4).setSize(4, 4);
    auto result = process({input}, config);
    REQUIRE(result->detections.size() == 2);
    REQUIRE_THAT(result->detections[0].boundingBox->center.x, WithinAbs(.5, 1e-6));
    REQUIRE_THAT(result->detections[1].boundingBox->center.x, WithinAbs(0, 1e-6));
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>{255, 255, 255, 255, 1, 0, 0, 255, 1, 0, 0, 255, 255, 255, 255, 255});
    input = message(8, 4);
    input->detections = {dai::ImgDetection(dai::RotatedRect(dai::Point2f(4, 2, false), dai::Size2f(2, 2, false), 90), .9f)};
    dai::Keypoint kp(dai::Point2f(.5f, .5f, true), .9f);
    input->detections[0].setKeypoints(std::vector<dai::Keypoint>{kp});
    input->setSegmentationMask({0, 255}, 2, 1);
    result = process({input}, {}, true);
    REQUIRE(result->detections[0].boundingBox->angle == 0);
    REQUIRE(result->detections[0].boundingBox->isNormalized());
    REQUIRE(result->detections[0].keypoints->keypoints[0].imageCoordinates.x == .5f);
    REQUIRE(result->getSegmentationMaskWidth() == 8);
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>{0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255,
                                                                0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255});
}

TEST_CASE("OV-8: confidence weighted average across standard angle boundary", "[ImgDetectionsFilter]") {
    auto a = message(), b = message();
    a->detections = {detection(256, 256, 40, 100, .5f, 0, 44)};
    b->detections = {detection(256, 256, 100, 40, .5f, 0, -44)};
    Config config;
    config.overlapMode = Mode::AVERAGE;
    const auto box = *process({a, b}, config, true)->detections[0].boundingBox;
    REQUIRE_THAT(box.angle, WithinAbs(45, 1e-5));
    REQUIRE_THAT(box.size.width * 512, WithinAbs(40, 1e-4));
    REQUIRE_THAT(box.size.height * 512, WithinAbs(100, 1e-4));
}

TEST_CASE("ER-5/ER-6/ER-10/OP-4: transformation and missing box errors", "[ImgDetectionsFilter]") {
    auto input = std::make_shared<dai::ImgDetections>();
    input->detections.emplace_back();
    REQUIRE(process({input})->detections.size() == 1);
    Config config;
    config.minArea = 1;
    REQUIRE_THROWS(process({input}, config));
    input->setTransformation(dai::ImgTransformation(8, 4));
    REQUIRE_THROWS(process({input}, config));
    config.minConfidence = .5f;
    REQUIRE(process({input}, config)->detections.empty());
    input->detections.clear();
    config = {};
    config.reference = dai::ImgTransformation(8, 4);
    input->transformation.reset();
    REQUIRE_THROWS(process({input}, config));
    input->setTransformation(*config.reference);
    input->transformation->setDistortionModel(dai::CameraModel::Equirectangular);
    REQUIRE_THROWS(process({input}, config));
    config.reference = input->transformation;
    REQUIRE(process({input}, config)->detections.empty());
}

TEST_CASE("MK-16/MK-17: invalid mask indices and >255 detections", "[ImgDetectionsFilter]") {
    auto input = message(2, 1);
    input->detections.emplace_back();
    input->setSegmentationMask({0, 4}, 2, 1);
    REQUIRE(*process({input})->getMaskData() == std::vector<std::uint8_t>{0, 255});
    input->detections.resize(256);
    input->detections[0].confidence = .5f;
    input->detections[255].confidence = .9f;
    input->setSegmentationMask({0, 254}, 2, 1);
    Config config;
    config.sortByConfidence = true;
    auto result = process({input}, config);
    REQUIRE(result->detections.size() == 256);
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>{1, 255});
}

TEST_CASE("Config wire roundtrip and strict range validation", "[ImgDetectionsFilter]") {
    Config config;
    config.labelsToKeep = std::vector<std::uint32_t>{1, 2};
    config.labelsToReject = std::vector<std::uint32_t>{2};
    config.setConfidenceRange(.2f, .8f).setSizeRange(2, 500).setWidthRange(1, 100).setHeightRange(2, 100);
    config.regionOfInterest = dai::Rect(1, 2, 30, 40, false);
    config.maxDetections = 3;
    config.sortByConfidence = true;
    config.overlapMode = Mode::AVERAGE;
    config.reference = dai::ImgTransformation(8, 4);
    config.setSequenceNum(42);
    config.setTimestamp(std::chrono::steady_clock::now());
    auto bytes = dai::StreamMessageParser::serializeMetadata(config);
    streamPacketDesc_t packet{};
    packet.data = bytes.data();
    packet.length = bytes.size();
    packet.fd = -1;
    auto copy = std::dynamic_pointer_cast<Config>(dai::StreamMessageParser::parseMessage(&packet));
    REQUIRE(copy);
    REQUIRE(dai::utility::serialize(config) == dai::utility::serialize(*copy));
    config.setConfidenceRange(.5f, .5f);
    REQUIRE_FALSE(config.validate());
    config = {};
    config.setSizeRange(2, 1);
    REQUIRE_FALSE(config.validate());
    config = {};
    config.setWidthRange(2, 1);
    REQUIRE_FALSE(config.validate());
    config = {};
    config.setHeightRange(2, 1);
    REQUIRE_FALSE(config.validate());
    config = {};
    config.overlapIouThreshold = 2;
    REQUIRE(config.validate());
}

TEST_CASE("IN/RD/S4/RT: linked keys, complete rounds, lexicographic order and runtime configs", "[ImgDetectionsFilter]") {
    RunningFilter filter({"cam2", "cam10"});
    filter.node->inputs["unused"];
    filter.node->initialConfig->reference = dai::ImgTransformation(512, 512);
    auto runtime = filter.node->inputConfig.createInputQueue();
    filter.pipeline.start();
    auto a = message(), b = message();
    a->detections = {detection(100, 100, 64, 64, .6f, 1), detection(300, 100, 64, 64, .9f, 2)};
    b->detections = {detection(100, 400, 64, 64, .7f, 3)};
    filter.inputs[0]->send(a);
    bool timedOut = false;
    REQUIRE_FALSE(filter.output->get<dai::ImgDetections>(std::chrono::milliseconds(30), timedOut));
    REQUIRE(timedOut);
    filter.inputs[1]->send(b);
    auto result = filter.receive();
    REQUIRE(result->detections[0].label == 3);
    REQUIRE(result->detections[1].label == 1);
    auto config = std::make_shared<Config>();
    config->labelsToKeep = std::vector<std::uint32_t>{2};
    filter.node->inputConfig.send(config);
    filter.inputs[0]->send(a);
    filter.inputs[1]->send(b);
    REQUIRE(filter.receive()->detections.size() == 1);
    config = std::make_shared<Config>();
    config->minConfidence = 2;
    filter.node->inputConfig.send(config);
    filter.inputs[0]->send(a);
    filter.inputs[1]->send(b);
    REQUIRE(filter.receive()->detections.size() == 1);  // Invalid runtime config keeps previous config.
    filter.node->inputConfig.send(std::make_shared<Config>());
    filter.inputs[0]->send(a);
    filter.inputs[1]->send(b);
    REQUIRE(filter.receive()->detections.size() == 3);  // Full replacement, latched config reference survives.
}

TEST_CASE("RF/S16: reference input latches and overrides runtime config reference", "[ImgDetectionsFilter]") {
    RunningFilter filter;
    auto refs = filter.node->inputReference.createInputQueue();
    auto configs = filter.node->inputConfig.createInputQueue();
    filter.pipeline.start();
    auto input = message(8, 4);
    input->detections = {detection(6, 2, 2, 2, .9f, 0, 0, 8, 4)};
    filter.inputs[0]->send(input);
    bool timedOut = false;
    REQUIRE_FALSE(filter.output->get<dai::ImgDetections>(std::chrono::milliseconds(50), timedOut));
    REQUIRE(timedOut);
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->getTransformation() = *input->getTransformation();
    filter.node->inputReference.send(frame);
    filter.inputs[0]->send(input);
    REQUIRE(filter.receive()->detections[0].boundingBox->center.x == .75f);
    frame = std::make_shared<dai::ImgFrame>();
    frame->getTransformation() = *input->getTransformation();
    frame->getTransformation().addCrop(4, 0, 4, 4).setSize(4, 4);
    filter.node->inputReference.send(frame);
    filter.inputs[0]->send(input);
    REQUIRE_THAT(filter.receive()->detections[0].boundingBox->center.x, WithinAbs(.5, 1e-6));
    auto config = std::make_shared<Config>();
    config->reference = input->transformation;
    filter.node->inputConfig.send(config);
    filter.node->inputReference.send(std::make_shared<dai::ImgFrame>());
    filter.inputs[0]->send(input);
    REQUIRE_THAT(filter.receive()->detections[0].boundingBox->center.x, WithinAbs(.5, 1e-6));
}

TEST_CASE("ER-1/ER-2/ER-3/CF-5: start rejects unusable configurations", "[ImgDetectionsFilter]") {
    SECTION("No linked inputs") {
        RunningFilter filter({});
        filter.node->inputs["unused"];
        REQUIRE_THROWS(filter.pipeline.start());
    }
    SECTION("Multiple inputs without reference") {
        RunningFilter filter({"a", "b"});
        REQUIRE_THROWS(filter.pipeline.start());
    }
    SECTION("Forced multi-input device placement") {
        RunningFilter filter({"a", "b"});
        filter.node->initialConfig->reference = dai::ImgTransformation(8, 4);
        filter.node->setRunOnHost(false);
        REQUIRE_THROWS(filter.pipeline.start());
    }
    SECTION("Invalid initial range") {
        RunningFilter filter;
        filter.node->initialConfig->setConfidenceRange(.5f, .5f);
        REQUIRE_THROWS(filter.pipeline.start());
    }
    SECTION("Unused key has no effect") {
        RunningFilter filter;
        filter.node->inputs["unused"];
        REQUIRE(filter.node->runOnHost());
        REQUIRE_NOTHROW(filter.pipeline.start());
    }
}

TEST_CASE("RM-2/RM-3/MK-8: panorama box and mask projection follows analytical rays", "[ImgDetectionsFilter]") {
    for(const auto model : {dai::CameraModel::Perspective, dai::CameraModel::Equirectangular, dai::CameraModel::Cylindrical}) {
        auto input = message(8, 4);
        const std::array<std::array<float, 3>, 3> intrinsics = {{{4, 0, 4}, {0, 4, 2}, {0, 0, 1}}};
        input->transformation->setIntrinsicMatrix(intrinsics);
        input->detections = {detection(4, 2, 2, 2, .9f, 0, 0, 8, 4)};
        input->setSegmentationMask({255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,   0,   255, 255, 255,
                                    255, 255, 255, 0,   0,   255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
                                   8,
                                   4);
        Config config;
        config.reference = *input->transformation;
        config.reference->setDistortionModel(model);
        auto result = process({input}, config);
        REQUIRE(result->detections.size() == 1);
        const auto box = result->detections[0].getBoundingBox();
        REQUIRE_THAT(box.center.x, WithinAbs(.5, 1e-5));
        REQUIRE_THAT(box.center.y, WithinAbs(.5, 1e-5));
        const auto mask = *result->getMaskData();
        for(std::size_t y = 0; y < 4; ++y)
            for(std::size_t x = 0; x < 8; ++x) {
                const float u = (x + .5f - 4) / 4, v = (y + .5f - 2) / 4;
                float px = x + .5f, py = y + .5f;
                if(model != dai::CameraModel::Perspective) {
                    px = 4 + 4 * std::tan(u);
                    py = 2 + 4 * (model == dai::CameraModel::Cylindrical ? v / std::cos(u) : std::tan(v) / std::cos(u));
                }
                const bool inside = px >= 3 && px < 5 && py >= 1 && py < 3;
                REQUIRE(mask[y * 8 + x] == (inside ? 0 : 255));
            }
    }
}

TEST_CASE("FL-1/FL-3: five-label keep and reject combinations", "[ImgDetectionsFilter]") {
    auto input = message();
    for(std::uint32_t label = 1; label <= 5; ++label) input->detections.push_back(detection(100, 100, 64, 64, .9f, label));
    Config config;
    config.labelsToKeep = std::vector<std::uint32_t>{1, 2, 3};
    config.labelsToReject = std::vector<std::uint32_t>{2, 4};
    auto result = process({input}, config);
    REQUIRE(result->detections.size() == 2);
    REQUIRE(result->detections[0].label == 1);
    REQUIRE(result->detections[1].label == 3);
    config.labelsToKeep.reset();
    result = process({input}, config);
    REQUIRE(result->detections.size() == 3);
    REQUIRE(result->detections[2].label == 5);
    config.labelsToReject.reset();
    config.labelsToKeep = std::vector<std::uint32_t>{1, 2, 3};
    REQUIRE(process({input}, config)->detections.size() == 3);
}

TEST_CASE("OV-2/OV-3/OV-5/OV-7/OV-9: rotated IoU, tie breaks and zero weights", "[ImgDetectionsFilter]") {
    auto a = message(512, 256), b = message(512, 256);
    a->detections = {detection(256, 128, 180, 10, 0, 0, 45, 512, 256)};
    b->detections = {detection(256, 128, 180, 10, 0, 0, -45, 512, 256)};
    REQUIRE(process({a, b}, {}, true)->detections.size() == 2);  // Outer rectangles overlap; rotated boxes barely intersect.
    a = message();
    b = message();
    a->detections = {detection(200, 200, 100, 100, 0)};
    b->detections = {detection(210, 200, 100, 100, 0), detection(190, 200, 100, 100, 0)};
    Config config;
    config.overlapMode = Mode::AVERAGE;
    auto result = process({a, b}, config, true);
    REQUIRE(result->detections.size() == 2);
    REQUIRE_THAT(result->detections[0].boundingBox->center.x * 512, WithinAbs(205, 1e-4));  // Equal IoU/confidence: first candidate wins.
    b->detections[1].confidence = .25f;
    result = process({a, b}, config, true);
    REQUIRE(result->detections.size() == 2);  // Higher-confidence B leader wins ties independently of A.
    a->detections.push_back(a->detections[0]);
    for(const auto mode : {Mode::OFF, Mode::NMS, Mode::AVERAGE}) {
        config.overlapMode = mode;
        REQUIRE(process({a}, config, true)->detections.size() == 2);
    }
    a->detections.resize(1);
    a->detections[0].boundingBox->size = {0, 0, true};
    b->detections = a->detections;
    config.overlapMode = Mode::NMS;
    REQUIRE(process({a, b}, config, true)->detections.empty());  // Remap removes boxes with no image area.
}

TEST_CASE("RM-7/RM-11/RM-12: unprojectable keypoints keep their slots and outside boxes drop", "[ImgDetectionsFilter]") {
    auto input = message(8, 4);
    input->transformation->setIntrinsicMatrix({{{4, 0, 4}, {0, 4, 2}, {0, 0, 1}}});
    dai::Extrinsics source({{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}, {10, 20, 30}, dai::CameraBoardSocket::CAM_A);
    source.toDeviceId = "rig";
    input->transformation->setExtrinsics(source);
    Config config;
    config.reference = dai::ImgTransformation(8, 4, {{{.5f, 0, 4}, {0, .5f, 2}, {0, 0, 1}}});
    dai::Extrinsics target({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A);
    target.toDeviceId = "rig";
    config.reference->setExtrinsics(target);
    input->detections = {detection(6, 2, 1, 1, .9f, 0, 0, 8, 4)};
    input->detections[0].setKeypoints(std::vector<dai::Keypoint>{dai::Keypoint(.25f, .5f, 0, .9f), dai::Keypoint(.75f, .5f, 0, .8f)},
                                      std::vector<dai::Edge>{{0, 1}});
    auto result = process({input}, config);
    REQUIRE(result->detections.size() == 1);
    REQUIRE(result->detections[0].keypoints->keypoints.size() == 2);
    REQUIRE(result->detections[0].keypoints->keypoints[0].confidence == 0);
    REQUIRE(result->detections[0].keypoints->keypoints[1].confidence == .8f);
    REQUIRE(result->detections[0].keypoints->edges.size() == 1);
    input->detections = {detection(2, 2, 1, 1, .9f, 0, 0, 8, 4)};
    REQUIRE(process({input}, config)->detections.empty());  // All corners behind the reference.
    input = message(8, 4);
    input->detections = {detection(3, 2, 2, 2, .9f, 0, 0, 8, 4), detection(4, 2, 2, 2, .8f, 0, 0, 8, 4)};
    config = {};
    config.reference = *input->transformation;
    config.reference->addCrop(4, 0, 4, 4).setSize(4, 4);
    result = process({input}, config);
    REQUIRE(result->detections.size() == 1);
    REQUIRE(result->detections[0].boundingBox->center.x == 0);
    REQUIRE(result->detections[0].xmin < 0);  // No clipping.
    target.toDeviceId = "other";
    config.reference->setExtrinsics(target);
    input->detections.clear();
    REQUIRE_THROWS(process({input}, config));  // Empty messages still validate coordinate-system compatibility.
}

TEST_CASE("S11: mask confidence ties, sorting, and unmasked detections", "[ImgDetectionsFilter]") {
    auto a = message(8, 4), b = message(8, 4);
    a->detections = {detection(2, 2, 4, 4, .5f, 0, 0, 8, 4)};
    b->detections = {detection(4, 2, 4, 4, .75f, 1, 0, 8, 4)};
    std::vector<std::uint8_t> am, bm;
    for(int y = 0; y < 4; ++y) {
        am.insert(am.end(), {0, 0, 0, 0, 255, 255, 255, 255});
        bm.insert(bm.end(), {255, 255, 0, 0, 0, 0, 255, 255});
    }
    a->setSegmentationMask(am, 8, 4);
    b->setSegmentationMask(bm, 8, 4);
    auto result = process({a, b}, {}, true);
    REQUIRE(*result->getMaskData()
            == std::vector<std::uint8_t>{0, 0, 1, 1, 1, 1, 255, 255, 0, 0, 1, 1, 1, 1, 255, 255, 0, 0, 1, 1, 1, 1, 255, 255, 0, 0, 1, 1, 1, 1, 255, 255});
    Config config;
    config.sortByConfidence = true;
    result = process({a, b}, config, true);
    REQUIRE(result->detections[0].label == 1);
    REQUIRE((*result->getMaskData())[0] == 1);
    REQUIRE((*result->getMaskData())[2] == 0);
    b->detections[0].confidence = .5f;
    result = process({a, b}, config, true);
    REQUIRE((*result->getMaskData())[2] == 0);
    a = message(8, 4);
    a->detections = {detection(2, 2, 4, 4, .9f, 0, 0, 8, 4)};
    result = process({a, b}, {}, true);
    REQUIRE((*result->getMaskData())[2] == 1);  // Unmasked high-confidence detection has no pixels.
}

TEST_CASE("RF-3/RT-2/RT-4/RT-6/RT-7: runtime config reference and live source transformations", "[ImgDetectionsFilter]") {
    RunningFilter filter;
    auto configs = filter.node->inputConfig.createInputQueue();
    filter.pipeline.start();
    auto input = message(8, 4);
    input->detections = {detection(6, 2, 2, 2, .9f, 0, 0, 8, 4)};
    filter.inputs[0]->send(input);
    REQUIRE(filter.receive()->detections[0].boundingBox->center.x == .75f);
    auto first = std::make_shared<Config>();
    first->labelsToKeep = std::vector<std::uint32_t>{};
    filter.node->inputConfig.send(first);
    auto last = std::make_shared<Config>();
    last->reference = *input->transformation;
    last->reference->addCrop(4, 0, 4, 4).setSize(4, 4);
    filter.node->inputConfig.send(last);
    filter.inputs[0]->send(input);
    auto baseline = filter.receive();
    REQUIRE(baseline->detections.size() == 1);
    REQUIRE_THAT(baseline->detections[0].boundingBox->center.x, WithinAbs(.5, 1e-6));
    filter.node->inputConfig.send(std::make_shared<Config>());
    auto changed = std::make_shared<dai::ImgDetections>(*input);
    changed->transformation->addCrop(1, 0, 7, 4).setSize(7, 4);
    filter.inputs[0]->send(changed);
    auto changedResult = filter.receive();
    REQUIRE(changedResult->detections.size() == 1);
    REQUIRE(changedResult->detections[0].boundingBox->center.x != .5f);
    filter.inputs[0]->send(input);
    REQUIRE(dai::utility::serialize(*filter.receive()) == dai::utility::serialize(*baseline));
}

TEST_CASE("IN-1/CF-1/CF-7/RD-5: typed inputs, unlimited defaults and empty rounds", "[ImgDetectionsFilter]") {
    RunningFilter filter;
    auto sync = filter.pipeline.create<dai::node::Sync>();
    REQUIRE_THROWS(sync->out.link(filter.node->inputs["cam"]));
    filter.pipeline.remove(sync);
    auto configs = filter.node->inputConfig.createInputQueue();
    filter.pipeline.start();
    auto input = std::make_shared<dai::ImgDetections>();
    for(const auto confidence : {-.25f, .5f, 1.25f}) {
        dai::ImgDetection item;
        item.confidence = confidence;
        input->detections.push_back(item);
    }
    filter.inputs[0]->send(input);
    REQUIRE(dai::utility::serialize(*filter.receive()) == dai::utility::serialize(*input));
    auto config = std::make_shared<Config>();
    config->setConfidenceRange(.2f);
    filter.node->inputConfig.send(config);
    filter.inputs[0]->send(input);
    REQUIRE(filter.receive()->detections.size() == 2);  // Default maximum imposes no limit.
    config = std::make_shared<Config>();
    config->maxDetections = 0;
    filter.node->inputConfig.send(config);
    filter.inputs[0]->send(input);
    REQUIRE(filter.receive()->detections.empty());
}

TEST_CASE("OV-7/RM-9/IN-5: average keeps the leader's label name and keypoints", "[ImgDetectionsFilter]") {
    auto a = message(), b = message();
    a->detections = {detection(200, 200, 100, 100, .8f)};
    b->detections = {detection(210, 200, 100, 100, .9f)};
    a->detections[0].labelName = "first camera";
    b->detections[0].labelName = "second camera";
    b->detections[0].setKeypoints(std::vector<dai::Keypoint>{dai::Keypoint(.3f, .4f, 0, .7f)});
    const auto originalA = dai::utility::serialize(*a), originalB = dai::utility::serialize(*b);
    Config config;
    config.overlapMode = Mode::AVERAGE;
    const auto result = process({a, b}, config, true);
    REQUIRE(result->detections.size() == 1);
    REQUIRE(result->detections[0].labelName == "second camera");
    REQUIRE(result->detections[0].confidence == .9f);
    REQUIRE(dai::utility::serialize(*result->detections[0].keypoints) == dai::utility::serialize(*b->detections[0].keypoints));
    REQUIRE(dai::utility::serialize(*a) == originalA);
    REQUIRE(dai::utility::serialize(*b) == originalB);
}

TEST_CASE("RM-5/RM-8: public rectangle remap canonicalizes sides and preserves subpixel units", "[ImgDetectionsFilter]") {
    const dai::ImgTransformation source(8, 4);
    const dai::RotatedRect rotated(dai::Point2f(4, 2, false), dai::Size2f(1, 2, false), 90);
    const auto canonical = source.remapRectTo(source, rotated);
    REQUIRE_THAT(canonical.angle, WithinAbs(0, 1e-5));
    REQUIRE_THAT(canonical.size.width, WithinAbs(2, 1e-5));
    REQUIRE_THAT(canonical.size.height, WithinAbs(1, 1e-5));
    REQUIRE_FALSE(canonical.isNormalized());
    auto reference = source;
    reference.addCrop(0, 0, 4, 4).setSize(4, 4);
    const auto tiny = dai::RotatedRect(dai::Point2f(.5f, .5f, false), dai::Size2f(.2f, .4f, false), 0).normalize(8, 4);
    const auto remapped = source.remapRectTo(reference, tiny);
    REQUIRE(remapped.isNormalized());
    REQUIRE_THAT(remapped.center.x, WithinAbs(.125, 1e-5));
    REQUIRE_THAT(remapped.center.y, WithinAbs(.125, 1e-5));
    REQUIRE_THAT(remapped.size.width, WithinAbs(.05, 1e-5));
    REQUIRE_THAT(remapped.size.height, WithinAbs(.1, 1e-5));
}
