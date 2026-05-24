#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("GPUStereo produces disparity on supported devices", "[GPUStereo][ondevice]") {
    std::shared_ptr<dai::Device> device;
    try {
        device = std::make_shared<dai::Device>();
    } catch(const std::exception& e) {
        SKIP(std::string("No device available: ") + e.what());
    }

    if(!device->hasGPU()) {
        SKIP("GPU not available on this device");
    }

    dai::Pipeline pipeline(device);

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30.0f);

    auto leftOut = monoLeft->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8);
    auto rightOut = monoRight->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8);

    auto gpu = pipeline.create<dai::node::GPUStereo>();
    leftOut->link(gpu->left);
    rightOut->link(gpu->right);
    gpu->setRectification(true);
    gpu->initialConfig->setConfidenceThreshold(25);

    auto disparityQ = gpu->disparity.createOutputQueue();

    pipeline.start();

    constexpr int kNumFrames = 10;
    for(int i = 0; i < kNumFrames; ++i) {
        auto frame = disparityQ->get<dai::ImgFrame>();
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getWidth() > 0);
        REQUIRE(frame->getHeight() > 0);
        REQUIRE_FALSE(frame->getData().empty());
    }
}
