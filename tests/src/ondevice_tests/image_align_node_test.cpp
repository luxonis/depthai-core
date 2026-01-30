#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <thread>

#include "depthai/depthai.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace {
void runImageAlignTest(bool useDepth, bool runOnHost, dai::ImgResizeMode resizeMode) {
    dai::Pipeline p;
    auto rgbCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto leftCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto align = p.create<dai::node::ImageAlign>();
    auto* rgbOut = rgbCam->requestOutput({1280, 640}, std::nullopt, resizeMode, std::nullopt, true);
    auto* leftOut = leftCam->requestOutput({1280, 800}, std::nullopt);
    auto* rightOut = rightCam->requestOutput({1280, 800}, std::nullopt);

    if(useDepth) {
        stereo = p.create<dai::node::StereoDepth>();
        leftOut->link(stereo->left);
        rightOut->link(stereo->right);
        stereo->depth.link(align->input);
    } else {
        leftOut->link(align->input);
        rightOut->createOutputQueue();  // TODO remove once left&rgb only streaming on RVC4 is supported
    }
    rgbOut->link(align->inputAlignTo);

    if(!useDepth) {
        align->initialConfig->staticDepthPlane = 0x5AB1;
    }
    if(runOnHost) {
        align->setRunOnHost(true);
    }

    auto alignedQueue = align->outputAligned.createOutputQueue();
    auto alignToQueue = rgbOut->createOutputQueue();
    p.start();

    auto alignToFrame = alignToQueue->get<dai::ImgFrame>();
    REQUIRE(alignToFrame != nullptr);
    const auto alignToIntrinsics = alignToFrame->transformation.getIntrinsicMatrix();

    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto aligned = alignedQueue->get<dai::ImgFrame>();
        REQUIRE(aligned != nullptr);
        REQUIRE(aligned->transformation.isAlignedTo(alignToFrame->transformation));
        REQUIRE(aligned->getInstanceNum() == alignToFrame->getInstanceNum());
    }
    p.stop();
}
}  // namespace

TEST_CASE("Test ImageAlign node image to image alignment") {
    bool useDepth = false;
    bool runOnHost = false;
    for(const auto resizeMode : {dai::ImgResizeMode::CROP, dai::ImgResizeMode::LETTERBOX, dai::ImgResizeMode::STRETCH}) {
        runImageAlignTest(useDepth, runOnHost, resizeMode);
    }
}

TEST_CASE("Test ImageAlign node depth to image alignment") {
    bool useDepth = true;
    bool runOnHost = false;
    for(const auto resizeMode : {dai::ImgResizeMode::CROP, dai::ImgResizeMode::LETTERBOX, dai::ImgResizeMode::STRETCH}) {
        runImageAlignTest(useDepth, runOnHost, resizeMode);
    }
}

TEST_CASE("Test ImageAlign node image to image alignment on host") {
    bool useDepth = false;
    bool runOnHost = true;
    for(const auto resizeMode : {dai::ImgResizeMode::CROP, dai::ImgResizeMode::LETTERBOX, dai::ImgResizeMode::STRETCH}) {
        runImageAlignTest(useDepth, runOnHost, resizeMode);
    }
}

TEST_CASE("Test ImageAlign node depth to image alignment on host") {
    bool useDepth = true;
    bool runOnHost = true;
    for(const auto resizeMode : {dai::ImgResizeMode::CROP, dai::ImgResizeMode::LETTERBOX, dai::ImgResizeMode::STRETCH}) {
        runImageAlignTest(useDepth, runOnHost, resizeMode);
    }
}
