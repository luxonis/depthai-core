#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("DetectionsCrop emits one paired crop per detection", "[DetectionsCrop][onhost]") {
    dai::Pipeline pipeline(false);
    auto detectionsCrop = pipeline.create<dai::node::DetectionsCrop>();
    detectionsCrop->setRunOnHost();
    REQUIRE(detectionsCrop->runOnHost());
    REQUIRE(detectionsCrop->cropConfigGenerator->runOnHost());
    REQUIRE(detectionsCrop->imageManip->runOnHost());
    REQUIRE(detectionsCrop->imageManip->inputConfig.getWaitForMessage());

    auto detectionsQueue = detectionsCrop->inputDetections.createInputQueue();
    auto imageQueue = detectionsCrop->inputImage.createInputQueue();
    auto outputQueue = detectionsCrop->out.createOutputQueue();

    auto detections = std::make_shared<dai::ImgDetections>();
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.2F, 0.25F, true), dai::Size2f(0.2F, 0.3F, true), 0.0F));
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.6F, 0.6F, true), dai::Size2f(0.4F, 0.5F, true), 0.0F));

    auto image = std::make_shared<dai::ImgFrame>();
    image->setSourceSize(10, 10);
    image->setSize(10, 10);
    image->setType(dai::ImgFrame::Type::RAW8);
    image->setData(std::vector<std::uint8_t>(100, 1));

    pipeline.start();
    detectionsQueue->send(detections);
    imageQueue->send(image);

    bool firstTimedOut = false;
    bool secondTimedOut = false;
    const auto first = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(1), firstTimedOut);
    const auto second = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(1), secondTimedOut);

    REQUIRE_FALSE(firstTimedOut);
    REQUIRE_FALSE(secondTimedOut);
    REQUIRE(first);
    REQUIRE(second);
    CHECK(first->getWidth() == 2);
    CHECK(first->getHeight() == 3);
    CHECK(second->getWidth() == 4);
    CHECK(second->getHeight() == 5);
}
