#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
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

/** Cameras for the live tests: CAM_A as the target with CAM_B and CAM_C when the device has all three. Otherwise the second
 *  camera is the target and the first the input; depth then comes from both cameras, aligned to the first. */
struct LiveSockets {
    dai::CameraBoardSocket target, left, right;
    bool threeCameras;
};

LiveSockets liveSockets(dai::Pipeline& pipeline) {
    const auto sockets = pipeline.getDefaultDevice()->getConnectedCameras();
    const auto has = [&](dai::CameraBoardSocket socket) { return std::find(sockets.begin(), sockets.end(), socket) != sockets.end(); };
    if(has(dai::CameraBoardSocket::CAM_A) && has(dai::CameraBoardSocket::CAM_B) && has(dai::CameraBoardSocket::CAM_C))
        return {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, true};
    if(sockets.size() < 2) SKIP("ImageAlign needs two cameras");
    return {sockets[1], sockets[0], sockets[1], false};
}

/** Keeps the device calibration when it relates the cameras, and otherwise sets a nominal one: a 71.86 degree FOV at each
 *  sensor's full size and 7.5 cm between neighboring cameras. */
void useCalibrationFor(dai::Pipeline& pipeline, const LiveSockets& sockets) {
    auto device = pipeline.getDefaultDevice();
    try {
        const auto calibration = device->getCalibration();
        calibration.getCameraExtrinsics(sockets.left, sockets.target);
        calibration.getCameraExtrinsics(sockets.left, sockets.right);
        return;
    } catch(const std::exception&) {
    }
    dai::CalibrationHandler calibration;
    for(const auto& camera : device->getConnectedCameraFeatures()) {
        const float width = static_cast<float>(camera.width), height = static_cast<float>(camera.height);
        const float focal = width / (2 * std::tan(71.86f * 3.14159265f / 360));
        calibration.setCameraIntrinsics(camera.socket, {{focal, 0, width / 2}, {0, focal, height / 2}, {0, 0, 1}}, camera.width, camera.height);
        calibration.setDistortionCoefficients(camera.socket, std::vector<float>(14, 0));
        calibration.setFov(camera.socket, 71.86f);
    }
    const std::vector<std::vector<float>> identity{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const std::vector<float> baseline{-7.5f, 0, 0};
    calibration.setCameraExtrinsics(sockets.left, sockets.right, identity, baseline, baseline);
    if(sockets.threeCameras) {
        const std::vector<float> toCenter{3.75f, 0, 0};
        calibration.setCameraExtrinsics(sockets.right, sockets.target, identity, toCenter, toCenter);
    }
    calibration.setStereoLeft(sockets.left, identity);
    calibration.setStereoRight(sockets.right, identity);
    pipeline.setCalibrationData(calibration);
}

void runImageAlignTest(bool useDepth, bool runOnHost, dai::ImgResizeMode resizeMode) {
    dai::Pipeline p;
    const auto sockets = liveSockets(p);
    useCalibrationFor(p, sockets);
    const bool rvc2ng = p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG;
    // RVC2NG cameras cannot undistort; the aligned output is the undistorted target camera either way.
    const bool undistort = !rvc2ng;
    auto rgbCam = p.create<dai::node::Camera>()->build(sockets.target);
    auto leftCam = p.create<dai::node::Camera>()->build(sockets.left);
    std::shared_ptr<dai::node::Camera> rightCam;
    if(sockets.threeCameras) {
        rightCam = p.create<dai::node::Camera>()->build(sockets.right);
    }
    std::shared_ptr<dai::node::StereoDepth> stereo;
    auto align = p.create<dai::node::ImageAlign>();
    auto* rgbOut = rgbCam->requestOutput({1280, 640}, std::nullopt, resizeMode, std::nullopt, undistort);
    // The host node has no NV16 path, which RVC2NG color cameras output by default.
    const auto inputType = runOnHost && rvc2ng ? std::optional<dai::ImgFrame::Type>(dai::ImgFrame::Type::NV12) : std::nullopt;
    auto* leftOut = leftCam->requestOutput({1280, 800}, inputType);
    // With two cameras, the target camera is also the right stereo camera.
    dai::Node::Output* rightOut = nullptr;
    if(rightCam || useDepth) rightOut = (rightCam ? rightCam : rgbCam)->requestOutput({1280, 800}, std::nullopt);

    if(useDepth) {
        stereo = p.create<dai::node::StereoDepth>();
        leftOut->link(stereo->left);
        rightOut->link(stereo->right);
        stereo->depth.link(align->input);
        if(!sockets.threeCameras) {
            // The target is the right camera, so depth stays on the left one.
            stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
        }
    } else {
        leftOut->link(align->input);
        if(rightOut) rightOut->createOutputQueue();  // TODO remove once left&rgb only streaming on RVC4 is supported
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
    auto undistortedTarget = alignToFrame->transformation;
    undistortedTarget.setDistortionCoefficients(std::vector<float>(undistortedTarget.getDistortionCoefficients().size(), 0.0f));

    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto aligned = alignedQueue->get<dai::ImgFrame>();
        REQUIRE(aligned != nullptr);
        requireAlignedFrameMetadata(*aligned, *alignToFrame);
        auto inputFrame = passthroughQueue->get<dai::ImgFrame>();
        REQUIRE(inputFrame != nullptr);
        requireInputMetadata(*aligned, *inputFrame);
        REQUIRE(aligned->transformation.isAlignedTo(undistortedTarget));
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
                                                   const dai::ImgTransformation& imageTransformation,
                                                   dai::Platform* platform = nullptr) {
    dai::Pipeline pipeline(!runOnHost);
    if(platform && !runOnHost) *platform = pipeline.getDefaultDevice()->getPlatform();
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(runOnHost);
    auto depthQueue = align->input.createInputQueue();
    auto imageQueue = align->inputAlignTo.createInputQueue();
    auto outputQueue = align->outputAligned.createOutputQueue();

    auto depth = makeRuntimeTransformationFrame(depthTransformation, dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Type::RAW16, 1);
    auto image = makeRuntimeTransformationFrame(imageTransformation, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::GRAY8, 1);
    depth->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(1100000000)));
    depth->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(1200000000)));
    depth->setTimestampSystem(
        std::chrono::system_clock::time_point(std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::nanoseconds(1300000000))));
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

    dai::Platform platform = dai::Platform::RVC4;
    const auto host = alignSyntheticDepth(true, depthTransformation, imageTransformation);
    const auto device = alignSyntheticDepth(false, depthTransformation, imageTransformation, &platform);
    const size_t imageBytes = width * height * 2;
    const auto hostData = host->getData();
    const auto deviceData = device->getData();
    REQUIRE(hostData.size() >= imageBytes);
    REQUIRE(deviceData.size() >= imageBytes);
    REQUIRE(std::any_of(hostData.begin(), hostData.begin() + imageBytes, [](uint8_t pixel) { return pixel != 0; }));
    if(platform != dai::Platform::RVC2NG) {
        REQUIRE(std::equal(hostData.begin(), hostData.begin() + imageBytes, deviceData.begin()));
        return;
    }
    // RVC2NG reprojects each depth sample to its nearest target pixel, while the host warp can place a sample one pixel
    // further along the baseline, or past the image edge. Both must cover the same pixels, and each host value must be
    // the device value of that pixel or of a horizontal neighbor.
    std::vector<uint16_t> hostDepth(width * height), deviceDepth(width * height);
    std::memcpy(hostDepth.data(), hostData.data(), imageBytes);
    std::memcpy(deviceDepth.data(), deviceData.data(), imageBytes);
    size_t coverageMismatches = 0, valueMismatches = 0;
    for(size_t y = 0; y < height; ++y) {
        for(size_t x = 0; x < width; ++x) {
            const auto value = hostDepth[y * width + x];
            coverageMismatches += (value == 0) != (deviceDepth[y * width + x] == 0);
            if(value == 0) continue;
            bool found = false;
            for(size_t nx = x == 0 ? 0 : x - 1; nx <= std::min(width - 1, x + 1); ++nx) found = found || deviceDepth[y * width + nx] == value;
            valueMismatches += !found;
        }
    }
    CAPTURE(coverageMismatches, valueMismatches);
    REQUIRE(coverageMismatches == 0);
    REQUIRE(valueMismatches == 0);
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
