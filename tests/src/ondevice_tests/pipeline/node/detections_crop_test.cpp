#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("DetectionsCrop crops detections on device", "[DetectionsCrop][ondevice]") {
    dai::Pipeline pipeline;
    auto detectionsCrop = pipeline.create<dai::node::DetectionsCrop>();

    REQUIRE_FALSE(detectionsCrop->cropConfigGenerator->runOnHost());
    REQUIRE_FALSE(detectionsCrop->imageManip->runOnHost());
    REQUIRE(detectionsCrop->imageManip->inputConfig.getWaitForMessage());

    auto detectionsQueue = detectionsCrop->inputDetections.createInputQueue();
    auto imageQueue = detectionsCrop->inputImage.createInputQueue();
    auto outputQueue = detectionsCrop->out.createOutputQueue();

    auto detections = std::make_shared<dai::ImgDetections>();
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.25F, 0.25F, true), dai::Size2f(0.25F, 0.25F, true), 0.0F));
    detections->detections.emplace_back(dai::RotatedRect(dai::Point2f(0.625F, 0.625F, true), dai::Size2f(0.5F, 0.5F, true), 0.0F));

    auto image = std::make_shared<dai::ImgFrame>();
    image->setSourceSize(640, 400);
    image->setSize(640, 400);
    image->setType(dai::ImgFrame::Type::RAW8);
    image->setSequenceNum(42);
    image->setData(std::vector<std::uint8_t>(640 * 400, 1));

    pipeline.start();
    detectionsQueue->send(detections);
    imageQueue->send(image);

    bool firstTimedOut = false;
    bool secondTimedOut = false;
    const auto first = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(5), firstTimedOut);
    const auto second = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(5), secondTimedOut);
    REQUIRE_FALSE(firstTimedOut);
    REQUIRE_FALSE(secondTimedOut);
    REQUIRE(first);
    REQUIRE(second);
    CHECK(first->getWidth() == 160);
    CHECK(first->getHeight() == 100);
    CHECK(second->getWidth() == 320);
    CHECK(second->getHeight() == 200);
    CHECK(first->getSequenceNum() == image->getSequenceNum());
    CHECK(second->getSequenceNum() == image->getSequenceNum());
}
