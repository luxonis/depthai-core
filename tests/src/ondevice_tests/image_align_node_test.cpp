#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <vector>

#include "depthai/depthai.hpp"

using namespace std;

namespace {
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
                                                              dai::ImgFrame::Type frameType,
                                                              int64_t sequenceNum) {
    const auto [width, height] = transformation.getSize();
    const auto [sourceWidth, sourceHeight] = transformation.getSourceSize();
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setSourceSize(sourceWidth, sourceHeight);
    frame->setWidth(width);
    frame->setHeight(height);
    frame->setType(frameType);
    const size_t bytesPerPixel = static_cast<size_t>(frame->getBytesPerPixel());
    frame->setStride(width * bytesPerPixel);
    frame->setInstanceNum(static_cast<uint32_t>(camera));
    frame->setSequenceNum(sequenceNum);
    std::vector<uint8_t> data(width * height * bytesPerPixel);
    for(size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<uint8_t>((i * 17) % 65536);
    }
    frame->setData(std::move(data));
    frame->setTransformation(transformation);
    REQUIRE(frame->validateTransformations());
    return frame;
}

std::shared_ptr<dai::ImgFrame> alignSyntheticDepth(bool runOnHost,
                                                   const dai::ImgTransformation& depthTransformation,
                                                   const dai::ImgTransformation& imageTransformation) {
    dai::Pipeline pipeline(!runOnHost);
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(runOnHost);
    auto depthQueue = align->input.createInputQueue();
    auto imageQueue = align->inputAlignTo.createInputQueue();
    auto outputQueue = align->outputAligned.createOutputQueue();

    auto depth = makeRuntimeTransformationFrame(depthTransformation, dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Type::RAW16, 1);
    auto image = makeRuntimeTransformationFrame(imageTransformation, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::GRAY8, 1);
    const auto [width, height] = depthTransformation.getSize();
    std::vector<uint8_t> depthPixels(width * height * 2);
    for(size_t y = 0; y < height; ++y) {
        for(size_t x = 0; x < width; ++x) {
            const uint16_t millimeters = static_cast<uint16_t>(1000 + 7 * x + 11 * y);
            const size_t offset = 2 * (y * width + x);
            depthPixels[offset] = static_cast<uint8_t>(millimeters);
            depthPixels[offset + 1] = static_cast<uint8_t>(millimeters >> 8);
        }
    }
    depth->setData(std::move(depthPixels));

    pipeline.start();
    imageQueue->send(image);
    depthQueue->send(depth);
    auto output = outputQueue->get<dai::ImgFrame>();
    REQUIRE(output != nullptr);
    requireAlignedFrameMetadata(*output, *image);
    requireInputMetadata(*output, *depth);
    REQUIRE(output->transformation.isAlignedTo(imageTransformation));
    pipeline.stop();
    return output;
}

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

TEST_CASE("Test ImageAlign synthetic depth to image host device parity") {
    constexpr size_t width = 64;
    constexpr size_t height = 48;
    const std::array<std::array<float, 3>, 3> intrinsics = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<std::vector<float>> identityRotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics depthExtrinsics(identityRotation, {70.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::Extrinsics imageExtrinsics(identityRotation, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation depthTransformation(width, height, intrinsics, dai::CameraModel::Perspective, {}, depthExtrinsics);
    const dai::ImgTransformation imageTransformation(width, height, intrinsics, dai::CameraModel::Perspective, {}, imageExtrinsics);

    const auto host = alignSyntheticDepth(true, depthTransformation, imageTransformation);
    const auto device = alignSyntheticDepth(false, depthTransformation, imageTransformation);
    const size_t imageBytes = width * height * 2;
    const auto hostData = host->getData();
    const auto deviceData = device->getData();
    REQUIRE(hostData.size() >= imageBytes);
    REQUIRE(deviceData.size() >= imageBytes);
    REQUIRE(std::any_of(hostData.begin(), hostData.begin() + imageBytes, [](uint8_t pixel) { return pixel != 0; }));
    REQUIRE(std::equal(hostData.begin(), hostData.begin() + imageBytes, deviceData.begin()));
}

// Feeds synthetic frames with hand-made ImgTransformations through ImageAlign and checks that a transformation change on
// either input is picked up at runtime: the aligned output has to follow the alignTo transformation (including its size)
// and the pixels have to change. Returning to the original transformations must reproduce the original output exactly.
void runImageAlignRuntimeTransformationTest(bool runOnHost, dai::ImgFrame::Type frameType, size_t width, size_t height) {
    // fx = width keeps the static depth plane shift (T * fx / plane) non-zero for the non-depth frame types.
    const float focalLength = static_cast<float>(width);
    const std::array<std::array<float, 3>, 3> intrinsics = {{{focalLength, 0.0f, width / 2.0f}, {0.0f, focalLength, height / 2.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<std::vector<float>> identityRotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    const dai::Extrinsics inputExtrinsics(identityRotation, {70.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::Extrinsics alignToExtrinsics(identityRotation, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation inputTransformation(width, height, intrinsics, dai::CameraModel::Perspective, {}, inputExtrinsics);
    const dai::ImgTransformation alignToTransformation(width, height, intrinsics, dai::CameraModel::Perspective, {}, alignToExtrinsics);

    auto changedAlignToTransformation = alignToTransformation;
    changedAlignToTransformation.addRotation(5.0f, {width / 2.0f, height / 2.0f});
    auto changedInputTransformation = inputTransformation;
    changedInputTransformation.addRotation(-3.0f, {width / 2.0f, height / 2.0f});
    // A resolution change of the alignTo frame changes the output resolution, so pools (and the device warps) have to be re-created.
    auto halfAlignToTransformation = alignToTransformation;
    halfAlignToTransformation.addScale(0.5f, 0.5f);
    REQUIRE_FALSE(changedAlignToTransformation.isEqualTransformation(alignToTransformation));
    REQUIRE_FALSE(changedInputTransformation.isEqualTransformation(inputTransformation));
    REQUIRE(halfAlignToTransformation.getSize() == std::make_pair(width / 2, height / 2));

    // The host variant needs no device: run it host-only so it does not depend on (re)connecting to one between cases.
    dai::Pipeline pipeline(!runOnHost);
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(runOnHost);
    align->input.setMaxSize(3);
    align->inputAlignTo.setMaxSize(3);
    if(frameType != dai::ImgFrame::Type::RAW16) {
        // Non-depth input: align with a static depth plane, same as the camera based tests above.
        align->initialConfig->staticDepthPlane = 0x5AB1;
    }
    auto inputQueue = align->input.createInputQueue(3, true);
    auto alignToQueue = align->inputAlignTo.createInputQueue(3, true);
    auto outputQueue = align->outputAligned.createOutputQueue(3, true);

    auto enqueue =
        [&](const dai::ImgTransformation& currentInputTransformation, const dai::ImgTransformation& currentAlignToTransformation, int64_t sequenceNum) {
            alignToQueue->send(makeRuntimeTransformationFrame(currentAlignToTransformation, dai::CameraBoardSocket::CAM_A, frameType, sequenceNum));
            inputQueue->send(makeRuntimeTransformationFrame(currentInputTransformation, dai::CameraBoardSocket::CAM_B, frameType, sequenceNum));
        };
    pipeline.start();

    auto requireAligned = [&](const std::shared_ptr<dai::ImgFrame>& aligned,
                              const dai::ImgTransformation& currentAlignToTransformation,
                              int64_t sequenceNum) {
        const auto [alignWidth, alignHeight] = currentAlignToTransformation.getSize();
        CAPTURE(sequenceNum);
        REQUIRE(aligned != nullptr);
        REQUIRE(aligned->getSequenceNum() == sequenceNum);
        REQUIRE(aligned->getInstanceNum() == static_cast<uint32_t>(dai::CameraBoardSocket::CAM_A));
        REQUIRE(aligned->getType() == frameType);
        CAPTURE(aligned->getWidth(), aligned->getHeight(), aligned->getSourceWidth(), aligned->getSourceHeight());
        CAPTURE(aligned->transformation.getSize(), aligned->transformation.getSourceSize(), aligned->transformation.isValid());
        REQUIRE(aligned->validateTransformations());
        REQUIRE(aligned->getWidth() == alignWidth);
        REQUIRE(aligned->getHeight() == alignHeight);
        REQUIRE(aligned->transformation.isAlignedTo(currentAlignToTransformation));

        const auto data = aligned->getData();
        const size_t imageSize = alignWidth * alignHeight * static_cast<size_t>(aligned->getBytesPerPixel());
        REQUIRE(data.size() >= imageSize);
        // Compare image bytes only: a pool buffer may be larger than the frame (row-aligned warp output on the device).
        std::vector<uint8_t> image(data.begin(), data.begin() + imageSize);
        REQUIRE(std::any_of(image.begin(), image.end(), [](uint8_t value) { return value != 0; }));
        return image;
    };

    int64_t sequenceNum = 0;
    auto sendAndRequireAligned = [&](const dai::ImgTransformation& currentInputTransformation,
                                     const dai::ImgTransformation& currentAlignToTransformation) {
        const auto [alignWidth, alignHeight] = currentAlignToTransformation.getSize();
        for(int attempt = 0; attempt < 10; ++attempt) {
            ++sequenceNum;
            CAPTURE(sequenceNum, attempt);
            enqueue(currentInputTransformation, currentAlignToTransformation, sequenceNum);
            auto aligned = outputQueue->get<dai::ImgFrame>();
            REQUIRE(aligned != nullptr);
            REQUIRE(aligned->getSequenceNum() == sequenceNum);
            if(aligned->getWidth() == alignWidth && aligned->getHeight() == alignHeight
               && aligned->transformation.isAlignedTo(currentAlignToTransformation)) {
                return requireAligned(aligned, currentAlignToTransformation, sequenceNum);
            }
        }
        FAIL("ImageAlign did not reconfigure to the new transformation");
        return std::vector<uint8_t>{};
    };

    const auto originalOutput = sendAndRequireAligned(inputTransformation, alignToTransformation);
    const auto changedAlignToOutput = sendAndRequireAligned(inputTransformation, changedAlignToTransformation);
    const auto changedInputOutput = sendAndRequireAligned(changedInputTransformation, changedAlignToTransformation);
    REQUIRE(changedAlignToOutput != originalOutput);
    REQUIRE(changedInputOutput != changedAlignToOutput);

    sendAndRequireAligned(inputTransformation, halfAlignToTransformation);

    // Back to the original transformations: nothing from the intermediate configurations (meshes, shift factor, pools)
    // may leak into the result, so the output must be identical to the very first one.
    const auto restoredOutput = sendAndRequireAligned(inputTransformation, alignToTransformation);
    REQUIRE(restoredOutput == originalOutput);

    pipeline.stop();
}
}  // namespace

// RAW16 (depth) is always aligned on the CPU. GRAY8 takes the hardware warp path on the device, and 640x400 is a
// resolution whose warp output buffers are row-aligned (larger than width * height), so both device code paths are covered.
TEST_CASE("Test ImageAlign runtime input transformations on host") {
    runImageAlignRuntimeTransformationTest(true, dai::ImgFrame::Type::RAW16, 64, 48);
    runImageAlignRuntimeTransformationTest(true, dai::ImgFrame::Type::GRAY8, 640, 400);
}

TEST_CASE("Test ImageAlign runtime input transformations") {
    runImageAlignRuntimeTransformationTest(false, dai::ImgFrame::Type::RAW16, 64, 48);
    runImageAlignRuntimeTransformationTest(false, dai::ImgFrame::Type::GRAY8, 640, 400);
}
