#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("Test Align node aligns ImgFrame to ImgFrame on device") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() == dai::Platform::RVC2) {
        SKIP("Align is not supported on the RVC2 platform");
    }

    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* sourceOutput = camera->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::LETTERBOX, std::nullopt, false);
    auto* alignToOutput = camera->requestOutput({1280, 720}, std::nullopt, dai::ImgResizeMode::STRETCH, std::nullopt, true);

    auto align = pipeline.create<dai::node::Align>();
    REQUIRE_FALSE(align->runOnHost());
    sourceOutput->link(align->input);
    alignToOutput->link(align->inputAlignTo);

    auto alignedQueue = align->outputAligned.createOutputQueue();
    auto alignToQueue = alignToOutput->createOutputQueue();

    pipeline.start();

    auto alignToFrame = alignToQueue->get<dai::ImgFrame>();
    REQUIRE(alignToFrame != nullptr);

    constexpr size_t N = 10;
    for(size_t i = 0; i < N; ++i) {
        auto aligned = alignedQueue->get<dai::ImgFrame>();
        REQUIRE(aligned != nullptr);
        REQUIRE(aligned->getWidth() == alignToFrame->getWidth());
        REQUIRE(aligned->getHeight() == alignToFrame->getHeight());
        REQUIRE(aligned->transformation.getSize() == alignToFrame->transformation.getSize());
    }

    pipeline.stop();
}
