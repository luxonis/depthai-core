#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"

TEST_CASE("Test ColorCamera node") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    auto leftCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto align = p.create<dai::node::ImageAlign>();

    auto leftOut = leftCam->requestOutput({1280, 800}, dai::ImgFrame::Type::NV12);
    auto rightOut = rightCam->requestOutput({1280, 800}, dai::ImgFrame::Type::NV12);

    leftOut->link(align->input);
    rightOut->link(align->inputAlignTo);

    align->initialConfig.staticDepthPlane = 0x5AB1;

    auto alignedQueue = align->outputAligned.createOutputQueue();

    p.start();

    // Retrieve N messages
    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto aligned = alignedQueue->get<dai::ImgFrame>();

        REQUIRE(aligned != nullptr);
    }
}
