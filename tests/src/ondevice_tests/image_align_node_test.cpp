#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace {
void runImageAlignTest(bool useDepth, bool runOnHost) {
    dai::Pipeline p;
    auto leftCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto align = p.create<dai::node::ImageAlign>();

    auto leftOut = leftCam->requestOutput({1280, 800});
    auto rightOut = rightCam->requestOutput({1280, 800});

    if(useDepth) {
        stereo = p.create<dai::node::StereoDepth>();
        leftOut->link(stereo->left);
        rightOut->link(stereo->right);
        stereo->depth.link(align->input);
    } else {
        leftOut->link(align->input);
    }
    rightOut->link(align->inputAlignTo);

    if(!useDepth) {
        align->initialConfig->staticDepthPlane = 0x5AB1;
    }
    if(runOnHost) {
        align->setRunOnHost(true);
    }

    auto alignedQueue = align->outputAligned.createOutputQueue();
    p.start();

    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto aligned = alignedQueue->get<dai::ImgFrame>();
        REQUIRE(aligned != nullptr);
    }
    p.stop();
}
}  // namespace

TEST_CASE("Test ImageAlign node image to image alignment") {
    bool useDepth = false;
    bool runOnHost = false;
    runImageAlignTest(useDepth, runOnHost);
}

TEST_CASE("Test ImageAlign node depth to image alignment") {
    bool useDepth = true;
    bool runOnHost = false;
    runImageAlignTest(useDepth, runOnHost);
}

TEST_CASE("Test ImageAlign node image to image alignment on host") {
    bool useDepth = false;
    bool runOnHost = true;
    runImageAlignTest(useDepth, runOnHost);
}

TEST_CASE("Test ImageAlign node depth to image alignment on host") {
    bool useDepth = true;
    bool runOnHost = true;
    runImageAlignTest(useDepth, runOnHost);
}
