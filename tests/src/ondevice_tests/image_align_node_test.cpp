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
void requireEqualSpecs(const dai::ImgFrame::Specs& actual, const dai::ImgFrame::Specs& expected) {
    REQUIRE(actual.type == expected.type);
    REQUIRE(actual.width == expected.width);
    REQUIRE(actual.height == expected.height);
    REQUIRE(actual.stride == expected.stride);
    REQUIRE(actual.bytesPP == expected.bytesPP);
    REQUIRE(actual.p1Offset == expected.p1Offset);
    REQUIRE(actual.p2Offset == expected.p2Offset);
    REQUIRE(actual.p3Offset == expected.p3Offset);
}

void requireInputMetadata(const dai::ImgFrame& aligned, const dai::ImgFrame& inputFrame) {
    REQUIRE(aligned.getType() == inputFrame.getType());
    REQUIRE(aligned.getBytesPerPixel() == inputFrame.getBytesPerPixel());
    REQUIRE(aligned.getSequenceNum() == inputFrame.getSequenceNum());
    REQUIRE(aligned.getTimestamp() == inputFrame.getTimestamp());
    REQUIRE(aligned.getTimestampDevice() == inputFrame.getTimestampDevice());
    REQUIRE(aligned.getTimestampSystem() == inputFrame.getTimestampSystem());
    REQUIRE(aligned.category == inputFrame.category);
    REQUIRE(aligned.event == inputFrame.event);
    REQUIRE(aligned.cam.exposureTimeUs == inputFrame.cam.exposureTimeUs);
    REQUIRE(aligned.cam.sensitivityIso == inputFrame.cam.sensitivityIso);
    REQUIRE(aligned.cam.lensPosition == inputFrame.cam.lensPosition);
    REQUIRE(aligned.cam.wbColorTemp == inputFrame.cam.wbColorTemp);
    REQUIRE(aligned.cam.lensPositionRaw == inputFrame.cam.lensPositionRaw);
    REQUIRE(aligned.cam.fsync == inputFrame.cam.fsync);
    REQUIRE(aligned.cam.sensorMode == inputFrame.cam.sensorMode);
    REQUIRE(aligned.cam.fps == inputFrame.cam.fps);
    REQUIRE(aligned.cam.sensorTemperatureC == inputFrame.cam.sensorTemperatureC);
}

void requireAlignedFrameMetadata(const dai::ImgFrame& aligned, const dai::ImgFrame& alignToFrame) {
    REQUIRE(aligned.getWidth() == alignToFrame.getWidth());
    REQUIRE(aligned.getHeight() == alignToFrame.getHeight());
    REQUIRE(aligned.getSourceWidth() == alignToFrame.getSourceWidth());
    REQUIRE(aligned.getSourceHeight() == alignToFrame.getSourceHeight());
    REQUIRE(aligned.validateTransformations());
    requireEqualSpecs(aligned.sourceFb, alignToFrame.sourceFb);

    if(aligned.getType() == dai::ImgFrame::Type::NV12) {
        REQUIRE(aligned.getPlaneHeight() == aligned.getHeight());
        REQUIRE(aligned.fb.p1Offset == 0);
        REQUIRE(aligned.fb.p2Offset == aligned.getStride() * aligned.getHeight());
        REQUIRE(aligned.fb.p3Offset == aligned.fb.p2Offset);
    } else if(aligned.getType() == dai::ImgFrame::Type::YUV420p) {
        REQUIRE(aligned.getPlaneHeight() == aligned.getHeight());
        REQUIRE(aligned.fb.p1Offset == 0);
        REQUIRE(aligned.fb.p2Offset == aligned.getStride() * aligned.getHeight());
        REQUIRE(aligned.fb.p3Offset == aligned.fb.p2Offset + (aligned.getStride() / 2) * (aligned.getHeight() / 2));
    }
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
    auto passthroughQueue = align->passthroughInput.createOutputQueue();
    auto alignToQueue = rgbOut->createOutputQueue();
    p.start();

    auto alignToFrame = alignToQueue->get<dai::ImgFrame>();
    REQUIRE(alignToFrame != nullptr);
    const auto alignToIntrinsics = alignToFrame->transformation.getIntrinsicMatrix();

    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto aligned = alignedQueue->get<dai::ImgFrame>();
        REQUIRE(aligned != nullptr);
        requireAlignedFrameMetadata(*aligned, *alignToFrame);
        auto inputFrame = passthroughQueue->get<dai::ImgFrame>();
        REQUIRE(inputFrame != nullptr);
        requireInputMetadata(*aligned, *inputFrame);
        REQUIRE(aligned->transformation.isAlignedTo(alignToFrame->transformation));
        REQUIRE(aligned->getInstanceNum() == alignToFrame->getInstanceNum());
    }
    p.stop();
}

std::shared_ptr<dai::ImgFrame> makeRuntimeTransformationFrame(const dai::ImgTransformation& transformation,
                                                              dai::CameraBoardSocket camera,
                                                              int64_t sequenceNum) {
    const auto [width, height] = transformation.getSize();
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setSourceSize(width, height);
    frame->setWidth(width);
    frame->setHeight(height);
    frame->setType(dai::ImgFrame::Type::RAW8);
    frame->setStride(width);
    frame->setInstanceNum(static_cast<uint32_t>(camera));
    frame->setSequenceNum(sequenceNum);
    std::vector<uint8_t> data(width * height);
    for(size_t y = 0; y < height; ++y) {
        for(size_t x = 0; x < width; ++x) {
            data[y * width + x] = static_cast<uint8_t>((x * 17 + y * 31) % 256);
        }
    }
    frame->setData(std::move(data));
    frame->setTransformation(transformation);
    REQUIRE(frame->validateTransformations());
    return frame;
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

TEST_CASE("Test ImageAlign device runtime input transformations") {
    constexpr size_t width = 64;
    constexpr size_t height = 48;
    const std::array<std::array<float, 3>, 3> intrinsics = {{{100.0f, 0.0f, width / 2.0f}, {0.0f, 100.0f, height / 2.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<std::vector<float>> identityRotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    const dai::Extrinsics inputExtrinsics(identityRotation, {7.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::Extrinsics alignToExtrinsics(identityRotation, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation inputTransformation(width, height, intrinsics, dai::CameraModel::Perspective, {}, inputExtrinsics);
    const dai::ImgTransformation alignToTransformation(width, height, intrinsics, dai::CameraModel::Perspective, {}, alignToExtrinsics);

    auto changedAlignToTransformation = alignToTransformation;
    changedAlignToTransformation.addRotation(5.0f, {width / 2.0f, height / 2.0f});
    auto changedInputTransformation = inputTransformation;
    changedInputTransformation.addRotation(-3.0f, {width / 2.0f, height / 2.0f});
    REQUIRE_FALSE(changedAlignToTransformation.isEqualTransformation(alignToTransformation));
    REQUIRE_FALSE(changedInputTransformation.isEqualTransformation(inputTransformation));

    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(false);
    // RAW8 is an image rather than a depth map, so provide the plane used by the device warp.
    align->initialConfig->staticDepthPlane = 1000;
    auto inputQueue = align->input.createInputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto outputQueue = align->outputAligned.createOutputQueue();
    pipeline.start();

    auto sendAndRequireAligned = [&](const dai::ImgTransformation& currentInputTransformation,
                                     const dai::ImgTransformation& currentAlignToTransformation,
                                     int64_t sequenceNum) {
        alignToQueue->send(makeRuntimeTransformationFrame(currentAlignToTransformation, dai::CameraBoardSocket::CAM_A, sequenceNum));
        inputQueue->send(makeRuntimeTransformationFrame(currentInputTransformation, dai::CameraBoardSocket::CAM_B, sequenceNum));

        auto aligned = outputQueue->get<dai::ImgFrame>();
        REQUIRE(aligned != nullptr);
        REQUIRE(aligned->getSequenceNum() == sequenceNum);
        REQUIRE(aligned->getInstanceNum() == static_cast<uint32_t>(dai::CameraBoardSocket::CAM_A));
        REQUIRE(aligned->transformation.isAlignedTo(currentAlignToTransformation));
        const auto data = aligned->getData();
        return std::vector<uint8_t>(data.begin(), data.end());
    };

    const auto originalOutput = sendAndRequireAligned(inputTransformation, alignToTransformation, 1);
    const auto changedAlignToOutput = sendAndRequireAligned(inputTransformation, changedAlignToTransformation, 2);
    const auto changedInputOutput = sendAndRequireAligned(changedInputTransformation, changedAlignToTransformation, 3);
    REQUIRE(changedAlignToOutput != originalOutput);
    REQUIRE(changedInputOutput != changedAlignToOutput);

    pipeline.stop();
}
