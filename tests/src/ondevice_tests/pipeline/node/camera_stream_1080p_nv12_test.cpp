#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Camera.hpp"

TEST_CASE("Camera streams 1080p NV12 frames for 5 minutes") {
    using namespace std::chrono_literals;
    constexpr auto streamDuration = 5min;
    constexpr auto frameTimeout = 10s;
    constexpr uint32_t width = 1920;
    constexpr uint32_t height = 1080;

    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto* output = camera->requestOutput(std::make_pair(width, height), dai::ImgFrame::Type::NV12);
    REQUIRE(output != nullptr);
    auto queue = output->createOutputQueue();

    p.start();

    uint64_t numFrames = 0;
    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < streamDuration) {
        bool hasTimedOut = false;
        auto frame = queue->get<dai::ImgFrame>(frameTimeout, hasTimedOut);
        REQUIRE_FALSE(hasTimedOut);
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(frame->getWidth() == width);
        REQUIRE(frame->getHeight() == height);
        REQUIRE(frame->getData().size() >= static_cast<size_t>(width) * height * 3 / 2);
        numFrames++;
    }

    const auto elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(numFrames > 0);
}
