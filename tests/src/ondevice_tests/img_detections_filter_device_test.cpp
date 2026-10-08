#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/depthai.hpp>

TEST_CASE("Filter config and keypoints roundtrip through a device", "[ImgDetectionsFilter][device]") {
    dai::Pipeline pipeline;
    auto echo = pipeline.create<dai::node::Script>();
    echo->setScript("while True:\n    node.outputs['out'].send(node.inputs['in'].get())\n");
    auto input = echo->inputs["in"].createInputQueue();
    auto output = echo->outputs["out"].createOutputQueue();
    pipeline.start();
    auto config = std::make_shared<dai::ImgDetectionsFilterConfig>();
    config->labelsToKeep = std::vector<std::uint32_t>{1, 2};
    config->labelsToReject = std::vector<std::uint32_t>{2};
    config->setConfidenceRange(.3f, .9f).setSizeRange(10, 400).setWidthRange(1, 100).setHeightRange(2, 100);
    config->maxDetections = 3;
    config->sortByConfidence = true;
    config->regionOfInterest = dai::Rect(0, 0, 100, 100, false);
    config->overlapMode = dai::ImgDetectionsFilterConfig::OverlapMode::AVERAGE;
    config->reference = dai::ImgTransformation(100, 100);
    config->setSequenceNum(42);
    config->setTimestamp(std::chrono::steady_clock::now());
    input->send(config);
    bool timeout = false;
    auto copy = output->get<dai::ImgDetectionsFilterConfig>(std::chrono::seconds(5), timeout);
    REQUIRE_FALSE(timeout);
    REQUIRE(copy);
    REQUIRE(dai::utility::serialize(*copy) == dai::utility::serialize(*config));

    auto detections = std::make_shared<dai::ImgDetections>();
    dai::ImgDetection detection;
    detection.setOuterBoundingBox(.1f, .2f, .3f, .4f);
    dai::Keypoint point(.2f, .3f, 0, .9f);
    detection.setKeypoints(std::vector<dai::Keypoint>{point});
    detections->detections.push_back(detection);
    input->send(detections);
    auto detectionCopy = output->get<dai::ImgDetections>(std::chrono::seconds(5), timeout);
    REQUIRE_FALSE(timeout);
    REQUIRE(detectionCopy);
    REQUIRE(dai::utility::serialize(*detectionCopy) == dai::utility::serialize(*detections));
}

TEST_CASE("Single input filtering has identical host and RVC4 device results", "[ImgDetectionsFilter][device]") {
    dai::Pipeline pipeline;
    auto automatic = pipeline.create<dai::node::ImgDetectionsFilter>();
    auto host = pipeline.create<dai::node::ImgDetectionsFilter>();
    host->setRunOnHost(true);
    automatic->initialConfig->setConfidenceRange(.5f);
    host->initialConfig->setConfidenceRange(.5f);
    auto input = automatic->inputs["cam"].createInputQueue();
    auto hostInput = host->inputs["cam"].createInputQueue();
    auto output = automatic->out.createOutputQueue();
    auto hostOutput = host->out.createOutputQueue();
    automatic->inputs["unused"];  // Unlinked keys cannot change device placement.
    REQUIRE(automatic->runOnHost() == (pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2));
    pipeline.start();
    auto detections = std::make_shared<dai::ImgDetections>();
    dai::ImgDetection a, b;
    a.confidence = .9f;
    b.confidence = .3f;
    detections->detections = {a, b};
    detections->setSegmentationMask({0, 1, 255, 0}, 4, 1);
    detections->setSequenceNum(12);
    detections->setTimestamp(std::chrono::steady_clock::now());
    input->send(detections);
    hostInput->send(detections);
    bool timeout = false;
    auto result = output->get<dai::ImgDetections>(std::chrono::seconds(5), timeout);
    REQUIRE_FALSE(timeout);
    REQUIRE(result);
    auto hostResult = hostOutput->get<dai::ImgDetections>(std::chrono::seconds(5), timeout);
    REQUIRE_FALSE(timeout);
    REQUIRE(hostResult);
    REQUIRE(result->detections.size() == 1);
    REQUIRE(*result->getMaskData() == std::vector<std::uint8_t>{0, 255, 255, 0});
    REQUIRE(dai::utility::serialize(*result) == dai::utility::serialize(*hostResult));
}
