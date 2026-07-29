#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
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

TEST_CASE("Test ImageAlign node same-camera transformation alignment") {
    dai::Pipeline p;
    auto align = p.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);

    auto inputQueue = align->input.createInputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto outputQueue = align->outputAligned.createOutputQueue();
    p.start();

    const std::array<std::array<float, 3>, 3> sourceIntrinsics = {{{400, 0, 2}, {0, 400, 2}, {0, 0, 1}}};
    const std::array<std::array<float, 3>, 3> targetIntrinsics = {{{200, 0, 1}, {0, 200, 1}, {0, 0, 1}}};
    dai::Extrinsics extrinsics;
    extrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_A;

    auto alignToFrame = std::make_shared<dai::ImgFrame>();
    alignToFrame->setWidth(2);
    alignToFrame->setHeight(2);
    alignToFrame->setType(dai::ImgFrame::Type::RAW8);
    alignToFrame->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_A));
    alignToFrame->setTransformation(dai::ImgTransformation(2, 2, targetIntrinsics, dai::CameraModel::Perspective, {}, extrinsics));
    alignToFrame->setData(std::vector<std::uint8_t>(4, 0));

    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputFrame->setWidth(4);
    inputFrame->setHeight(4);
    inputFrame->setType(dai::ImgFrame::Type::RAW16);
    inputFrame->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_A));
    inputFrame->setTransformation(dai::ImgTransformation(4, 4, sourceIntrinsics, dai::CameraModel::Perspective, {}, extrinsics));
    const std::vector<std::uint16_t> inputData = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    std::vector<std::uint8_t> inputBytes(inputData.size() * sizeof(std::uint16_t));
    std::memcpy(inputBytes.data(), inputData.data(), inputBytes.size());
    inputFrame->setData(std::move(inputBytes));

    alignToQueue->send(alignToFrame);
    inputQueue->send(inputFrame);

    auto aligned = outputQueue->get<dai::ImgFrame>();
    REQUIRE(aligned != nullptr);
    REQUIRE(aligned->getWidth() == alignToFrame->getWidth());
    REQUIRE(aligned->getHeight() == alignToFrame->getHeight());
    REQUIRE(aligned->transformation.isAlignedTo(alignToFrame->transformation));
    REQUIRE(aligned->getInstanceNum() == alignToFrame->getInstanceNum());
    const std::vector<std::uint16_t> expectedData = {1, 3, 9, 11};
    std::vector<std::uint16_t> alignedData(expectedData.size());
    REQUIRE(aligned->getData().size() == alignedData.size() * sizeof(std::uint16_t));
    std::memcpy(alignedData.data(), aligned->getData().data(), aligned->getData().size());
    REQUIRE(alignedData == expectedData);
    p.stop();
}
