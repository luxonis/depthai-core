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
constexpr auto kRuntimeCalibrationFrameTimeout = 180;
constexpr auto kRuntimeCalibrationShiftPx = 40.0f;
constexpr auto kRuntimeCalibrationVerificationFrames = 20;

bool approxEqual(float a, float b, float absTol = 1e-4f, float relTol = 1e-4f) {
    return std::abs(a - b) <= (absTol + relTol * std::max(std::abs(a), std::abs(b)));
}

bool sameIntrinsics(const std::array<std::array<float, 3>, 3>& lhs, const std::array<std::array<float, 3>, 3>& rhs) {
    for(size_t i = 0; i < lhs.size(); ++i) {
        for(size_t j = 0; j < lhs[i].size(); ++j) {
            if(!approxEqual(lhs[i][j], rhs[i][j])) {
                return false;
            }
        }
    }
    return true;
}

std::shared_ptr<dai::ImgFrame> waitForFrameAlignedTo(
    const std::shared_ptr<dai::MessageQueue>& queue, const dai::ImgTransformation& transformation, size_t maxFrames = kRuntimeCalibrationFrameTimeout) {
    for(size_t i = 0; i < maxFrames; ++i) {
        auto frame = queue->get<dai::ImgFrame>();
        if(frame != nullptr && frame->transformation.isAlignedTo(transformation)) {
            return frame;
        }
    }
    return nullptr;
}

std::shared_ptr<dai::ImgFrame> waitForFrameWithChangedTransform(
    const std::shared_ptr<dai::MessageQueue>& queue, const dai::ImgTransformation& baseline, size_t maxFrames = kRuntimeCalibrationFrameTimeout) {
    for(size_t i = 0; i < maxFrames; ++i) {
        auto frame = queue->get<dai::ImgFrame>();
        if(frame != nullptr && !frame->transformation.isAlignedTo(baseline)) {
            return frame;
        }
    }
    return nullptr;
}

std::shared_ptr<dai::ImgFrame> waitForNextFrame(const std::shared_ptr<dai::MessageQueue>& queue, size_t maxFrames = kRuntimeCalibrationFrameTimeout) {
    for(size_t i = 0; i < maxFrames; ++i) {
        auto frame = queue->get<dai::ImgFrame>();
        if(frame != nullptr) {
            return frame;
        }
    }
    return nullptr;
}

dai::CalibrationHandler makeShiftedCalibration(const dai::CalibrationHandler& base, std::tuple<int, int> outputSize, float shiftPx) {
    dai::CalibrationHandler updated = base;
    auto intrinsics = updated.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, outputSize);
    intrinsics[0][2] += shiftPx;
    intrinsics[1][2] += shiftPx * 0.5f;
    updated.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, intrinsics, outputSize);
    return updated;
}

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

void runImageAlignRuntimeCalibrationTest(bool useDepth) {
    constexpr auto rgbSize = std::pair<int, int>{1280, 640};

    dai::Pipeline p;
    auto rgbCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto leftCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto align = p.create<dai::node::ImageAlign>();

    auto* rgbOut = rgbCam->requestOutput(rgbSize, std::nullopt, dai::ImgResizeMode::CROP, std::nullopt, true);
    auto* leftOut = leftCam->requestOutput({1280, 800}, std::nullopt);
    auto* rightOut = rightCam->requestOutput({1280, 800}, std::nullopt);

    if(useDepth) {
        stereo = p.create<dai::node::StereoDepth>();
        leftOut->link(stereo->left);
        rightOut->link(stereo->right);
        stereo->depth.link(align->input);
    } else {
        leftOut->link(align->input);
        rightOut->createOutputQueue();  // Keep both mono streams active on platforms that require stereo pair streaming.
        align->initialConfig->staticDepthPlane = 0x5AB1;
    }
    rgbOut->link(align->inputAlignTo);

    auto alignedQueue = align->outputAligned.createOutputQueue();
    auto alignToQueue = rgbOut->createOutputQueue();
    auto device = p.getDefaultDevice();
    p.start();

    auto baselineAlignTo = alignToQueue->get<dai::ImgFrame>();
    REQUIRE(baselineAlignTo != nullptr);

    auto baselineAligned = waitForFrameAlignedTo(alignedQueue, baselineAlignTo->transformation);
    REQUIRE(baselineAligned != nullptr);
    REQUIRE(baselineAligned->getInstanceNum() == baselineAlignTo->getInstanceNum());

    auto originalCalibration = device->getCalibration();
    auto shiftedCalibration = makeShiftedCalibration(originalCalibration, rgbSize, kRuntimeCalibrationShiftPx);
    device->setCalibration(shiftedCalibration);

    auto shiftedAlignTo = waitForFrameWithChangedTransform(alignToQueue, baselineAlignTo->transformation, kRuntimeCalibrationVerificationFrames);
    if(shiftedAlignTo == nullptr) {
        shiftedAlignTo = waitForNextFrame(alignToQueue);
    }
    REQUIRE(shiftedAlignTo != nullptr);

    auto shiftedAligned = waitForFrameAlignedTo(alignedQueue, shiftedAlignTo->transformation);
    REQUIRE(shiftedAligned != nullptr);

    device->setCalibration(originalCalibration);

    auto revertedAlignTo = waitForFrameAlignedTo(alignToQueue, baselineAlignTo->transformation, kRuntimeCalibrationVerificationFrames);
    if(revertedAlignTo == nullptr) {
        revertedAlignTo = waitForNextFrame(alignToQueue);
    }
    REQUIRE(revertedAlignTo != nullptr);

    for(size_t i = 0; i < kRuntimeCalibrationVerificationFrames; ++i) {
        auto revertedAligned = alignedQueue->get<dai::ImgFrame>();
        REQUIRE(revertedAligned != nullptr);
        REQUIRE(revertedAligned->transformation.isAlignedTo(revertedAlignTo->transformation));
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

TEST_CASE("Test ImageAlign node updates aligned transform after runtime calibration change") {
    runImageAlignRuntimeCalibrationTest(true);
}

TEST_CASE("Test ImageAlign node resets image-to-image alignment after runtime calibration change") {
    runImageAlignRuntimeCalibrationTest(false);
}
