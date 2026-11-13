#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <type_traits>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/utility/ImageAlignDepth.hpp"
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/imgproc.hpp>

    #include "pipeline/utilities/Alignment/ImageAlignCpu.hpp"
#endif

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

template <typename AlignNode = dai::node::ImageAlign>
std::shared_ptr<dai::ImgFrame> alignSyntheticDepth(bool runOnHost,
                                                   const dai::ImgTransformation& depthTransformation,
                                                   const dai::ImgTransformation& imageTransformation,
                                                   const std::vector<uint8_t>& inputDepthPixels = {},
                                                   dai::Backend backend = dai::Backend::AUTO) {
    dai::Pipeline pipeline(!runOnHost);
    if constexpr(std::is_same_v<AlignNode, dai::node::Align>) {
        if(!runOnHost && pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
            SKIP("Align has no RVC2 firmware implementation");
        }
    }
    auto align = pipeline.create<AlignNode>();
    if constexpr(std::is_same_v<AlignNode, dai::node::ImageAlign>) align->setBackend(backend);
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
    if(!inputDepthPixels.empty()) {
        REQUIRE(inputDepthPixels.size() == depthPixels.size());
        depthPixels = inputDepthPixels;
    }
    depth->setData(std::move(depthPixels));

    pipeline.start();
    imageQueue->send(image);
    depthQueue->send(depth);
    auto output = outputQueue->template get<dai::ImgFrame>();
    REQUIRE(output != nullptr);
    requireAlignedFrameMetadata(*output, *image);
    requireInputMetadata(*output, *depth);
    REQUIRE(output->transformation.isAlignedTo(imageTransformation));
    pipeline.stop();
    return output;
}

namespace {
template <typename AlignNode>
void requireProjectedDepthPoints(bool runOnHost, dai::Backend backend = dai::Backend::AUTO) {
    CAPTURE(backend);
    constexpr int width = 320, height = 240;
    constexpr float fx = 250.0f, fy = 250.0f, cx = 159.5f, cy = 119.5f;
    const std::array<std::array<float, 3>, 3> intrinsics = {{{fx, 0, cx}, {0, fy, cy}, {0, 0, 1}}};
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const dai::Extrinsics targetExtrinsics(identity, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation targetTransform(width, height, intrinsics, dai::CameraModel::Perspective, {}, targetExtrinsics);
    const std::vector<std::array<float, 3>> points = {{{-200, -120, 1000}}, {{150, 80, 1200}}, {{-140, 90, 700}}, {{250, -80, 1000}}};
    const auto project = [&](const std::array<float, 3>& point) { return std::array<float, 2>{fx * point[0] / point[2] + cx, fy * point[1] / point[2] + cy}; };
    for(const auto& geometry : {std::array<float, 4>{10, 75, 50, 0}, {0, 75, 0, 0}, {-7, -75, -30, 0}, {10, 75, 50, 6}, {0, 75, 0, 6}}) {
        const float angle = geometry[0] * 3.14159265358979323846f / 180.0f;
        const float c = std::cos(angle), sn = std::sin(angle);
        const float pitch = geometry[3] * 3.14159265358979323846f / 180.0f;
        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const std::vector<std::vector<float>> rotation = {{c, sn * sp, sn * cp}, {0, cp, -sp}, {-sn, c * sp, c * cp}};
        const dai::Extrinsics sourceExtrinsics(rotation, {geometry[1], 0, geometry[2]}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation sourceTransform(width, height, intrinsics, dai::CameraModel::Perspective, {}, sourceExtrinsics);
        std::vector<uint8_t> pixels(width * height * 2, 0);
        for(const auto& point : points) {
            const auto sourcePixel = project(point);
            const int u = std::lround(sourcePixel[0]), v = std::lround(sourcePixel[1]);
            REQUIRE(u > 1);
            REQUIRE(u < width - 2);
            REQUIRE(v > 1);
            REQUIRE(v < height - 2);
            // A small footprint survives the two nearest-neighbor remaps around the projected point.
            for(int dy = -1; dy <= 1; ++dy) {
                for(int dx = -1; dx <= 1; ++dx) {
                    const auto offset = 2 * ((v + dy) * width + u + dx);
                    const auto depth = static_cast<uint16_t>(point[2]);
                    pixels[offset] = static_cast<uint8_t>(depth);
                    pixels[offset + 1] = static_cast<uint8_t>(depth >> 8);
                }
            }
        }
        const auto output = alignSyntheticDepth<AlignNode>(runOnHost, sourceTransform, targetTransform, pixels, backend);
        const auto data = output->getData();
        const auto stride = output->getStride();
        REQUIRE(stride >= width * 2);
        REQUIRE(data.size() >= stride * height);
        for(const auto& point : points) {
            const float rotatedZ = sp * point[1] + cp * point[2];
            const std::array<float, 3> targetPoint = {
                c * point[0] + sn * rotatedZ + geometry[1], cp * point[1] - sp * point[2], -sn * point[0] + c * rotatedZ + geometry[2]};
            REQUIRE(targetPoint[2] > 0);
            const auto targetPixel = project(targetPoint);
            CAPTURE(geometry, point, targetPoint, targetPixel);
            float closestDistanceSquared = std::numeric_limits<float>::infinity();
            uint16_t closestDepth = 0;
            for(int y = std::lround(targetPixel[1]) - 3; y <= std::lround(targetPixel[1]) + 3; ++y) {
                for(int x = std::lround(targetPixel[0]) - 3; x <= std::lround(targetPixel[0]) + 3; ++x) {
                    if(x < 0 || x >= width || y < 0 || y >= height) continue;
                    const auto offset = y * stride + x * 2;
                    const uint16_t depth = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
                    if(depth == 0) continue;
                    const float dx = x - targetPixel[0], dy = y - targetPixel[1];
                    const float distanceSquared = dx * dx + dy * dy;
                    if(distanceSquared < closestDistanceSquared) {
                        closestDistanceSquared = distanceSquared;
                        closestDepth = depth;
                    }
                }
            }
            REQUIRE(closestDistanceSquared <= 4.0f);
            REQUIRE(std::abs(static_cast<float>(closestDepth) - targetPoint[2]) <= 3.0f);
            // Equivalent rectified disparity in pixels, using the same baseline for both depths.
            const float baseline = std::hypot(geometry[1], geometry[2]);
            const float disparityProjected = fx * baseline / targetPoint[2];
            const float disparityAligned = fx * baseline / closestDepth;
            const float disparityDifference = disparityProjected - disparityAligned;
            CAPTURE(disparityProjected, disparityAligned, disparityDifference);
            REQUIRE(std::abs(disparityDifference) <= 0.2f);
        }
    }
}
}  // namespace

TEST_CASE("ImageAlign synthetic 3D point projections on host") {
    for(auto backend : {dai::Backend::AUTO, dai::Backend::CPU, dai::Backend::GPU}) requireProjectedDepthPoints<dai::node::ImageAlign>(true, backend);
}
TEST_CASE("ImageAlign synthetic 3D point projections on device") {
    const auto platform = dai::Device().getPlatform();
    for(auto backend : {dai::Backend::AUTO, dai::Backend::CPU, dai::Backend::GPU}) {
        if(platform == dai::Platform::RVC2 && backend != dai::Backend::AUTO) continue;
        requireProjectedDepthPoints<dai::node::ImageAlign>(false, backend);
    }
}
TEST_CASE("Align synthetic 3D point projections on host") {
    requireProjectedDepthPoints<dai::node::Align>(true);
}
TEST_CASE("Align synthetic 3D point projections on device") {
    requireProjectedDepthPoints<dai::node::Align>(false);
}

TEST_CASE("ImageAlign and Align keep independent depth caches on device") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        SKIP("Align has no RVC2 firmware implementation");
    }
    auto imageAlign = pipeline.create<dai::node::ImageAlign>();
    auto align = pipeline.create<dai::node::Align>();
    imageAlign->setRunOnHost(false);
    align->setRunOnHost(false);
    const std::array sourceQueues = {imageAlign->input.createInputQueue(), align->input.createInputQueue()};
    const std::array targetQueues = {imageAlign->inputAlignTo.createInputQueue(), align->inputAlignTo.createInputQueue()};
    const std::array outputQueues = {imageAlign->outputAligned.createOutputQueue(), align->outputAligned.createOutputQueue()};
    const std::array<std::array<float, 3>, 3> intrinsics = {{{250, 0, 159.5f}, {0, 250, 119.5f}, {0, 0, 1}}};
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const dai::Extrinsics targetExtrinsics(identity, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation targetTransform(320, 240, intrinsics, dai::CameraModel::Perspective, {}, targetExtrinsics);
    pipeline.start();
    for(int repeat = 0; repeat < 6; ++repeat) {
        for(size_t index = 0; index < sourceQueues.size(); ++index) {
            const bool positive = (index == 0) != (repeat >= 3);
            const float angle = (positive ? 10.0f : -7.0f) * 3.14159265358979323846f / 180.0f;
            const float sn = std::sin(angle), c = std::cos(angle);
            const float tx = positive ? 75.0f : -75.0f, tz = positive ? 50.0f : -30.0f;
            const dai::Extrinsics extrinsics({{c, 0, sn}, {0, 1, 0}, {-sn, 0, c}}, {tx, 0, tz}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
            const dai::ImgTransformation sourceTransform(320, 240, intrinsics, dai::CameraModel::Perspective, {}, extrinsics);
            auto source = makeRuntimeTransformationFrame(sourceTransform, dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Type::RAW16, repeat);
            std::vector<uint8_t> pixels(320 * 240 * 2);
            for(size_t offset = 0; offset < pixels.size(); offset += 2) {
                pixels[offset] = 1000 & 0xff;
                pixels[offset + 1] = 1000 >> 8;
            }
            source->setData(std::move(pixels));
            targetQueues[index]->send(makeRuntimeTransformationFrame(targetTransform, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::GRAY8, repeat));
            sourceQueues[index]->send(source);
            auto output = outputQueues[index]->get<dai::ImgFrame>();
            REQUIRE(output != nullptr);
            REQUIRE(output->getSequenceNum() == repeat);
            const auto data = output->getData();
            for(int u : {130, 160, 190}) {
                const size_t offset = 120 * output->getStride() + u * 2;
                const uint16_t actual = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
                const float expected = (1000 + sn * tx + c * tz) / (sn * (u - 159.5f) / 250 + c);
                CAPTURE(repeat, index, u, actual, expected);
                REQUIRE(std::abs(actual - expected) <= 0.51f);
            }
        }
    }
    pipeline.stop();
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

namespace {
struct SyntheticAlignmentCase {
    int width, height;
    float yaw, baseline, axialTranslation;
};

void requireSyntheticPixels(const dai::ImgFrame& expected, const dai::ImgFrame& actual, const dai::ImgFrame* source = nullptr) {
    const auto reference = expected.getData(), data = actual.getData();
    const size_t bpp = static_cast<size_t>(expected.getBytesPerPixel());
    const int width = expected.getWidth(), height = expected.getHeight();
    REQUIRE(actual.getWidth() == expected.getWidth());
    REQUIRE(actual.getHeight() == expected.getHeight());
    REQUIRE(actual.getType() == expected.getType());
    const auto depthAt = [bpp](const auto& pixels, size_t stride, int x, int y) {
        const size_t offset = y * stride + x * bpp;
        return uint16_t(pixels[offset] | (bpp == 2 ? uint16_t(pixels[offset + 1]) << 8 : 0));
    };
    size_t mismatches = 0, unmatched = 0;
    int firstX = 0, firstY = 0;
    uint16_t firstExpected = 0, firstActual = 0;
    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
            const uint16_t wanted = depthAt(reference, expected.getStride(), x, y), got = depthAt(data, actual.getStride(), x, y);
            if(wanted == got) continue;
            if(mismatches++ == 0) {
                firstX = x;
                firstY = y;
                firstExpected = wanted;
                firstActual = got;
            }
            if(!source) continue;
            // Bound the expensive independent projection check if a regression changes many pixels.
            if(mismatches > size_t(width) * height / 10000) {
                CAPTURE(mismatches, firstX, firstY, firstExpected, firstActual);
                FAIL("RAW16 agreement fell below 99.99 percent");
            }
            // Validate both possible winners against independently projected input points.
            const auto input = source->getData();
            const auto sourceK = source->transformation.getIntrinsicMatrix();
            const auto targetK = expected.transformation.getIntrinsicMatrix();
            const auto rotation = source->transformation.getRotationMatrixTo(expected.transformation);
            const auto translation = source->transformation.getTranslationVectorTo(expected.transformation, false, dai::LengthUnit::MILLIMETER);
            const float rayX = (x - targetK[0][2]) / targetK[0][0];
            const float rayY = (y - targetK[1][2]) / targetK[1][1];
            // Intersect the output ray with each candidate's source-Z plane. Comparing
            // Z at the projected source ray would penalize the allowed pixel displacement.
            const float normalDotRay = rotation[0][2] * rayX + rotation[1][2] * rayY + rotation[2][2];
            const float normalDotTranslation = rotation[0][2] * translation[0] + rotation[1][2] * translation[1] + rotation[2][2] * translation[2];
            if(!std::isfinite(normalDotRay) || normalDotRay <= 1e-6f) {
                ++unmatched;
                continue;
            }
            bool foundExpected = wanted == 0, foundActual = got == 0;
            for(unsigned int v = 0; v < source->getHeight() && (!foundExpected || !foundActual); ++v) {
                for(unsigned int u = 0; u < source->getWidth() && (!foundExpected || !foundActual); ++u) {
                    const float z = depthAt(input, source->getStride(), u, v);
                    if(z == 0) continue;
                    const float px = (u - sourceK[0][2]) * z / sourceK[0][0];
                    const float py = (v - sourceK[1][2]) * z / sourceK[1][1];
                    const float targetZ = rotation[2][0] * px + rotation[2][1] * py + rotation[2][2] * z + translation[2];
                    if(targetZ <= 0) continue;
                    const float targetX = rotation[0][0] * px + rotation[0][1] * py + rotation[0][2] * z + translation[0];
                    const float targetY = rotation[1][0] * px + rotation[1][1] * py + rotation[1][2] * z + translation[1];
                    const float dx = targetK[0][0] * targetX / targetZ + targetK[0][2] - x;
                    const float dy = targetK[1][1] * targetY / targetZ + targetK[1][2] - y;
                    if(dx * dx + dy * dy > 4.0f) continue;
                    const float outputRayZ = (z + normalDotTranslation) / normalDotRay;
                    if(!(outputRayZ >= 0.5f && outputRayZ < 65535.5f)) continue;
                    foundExpected |= std::abs(outputRayZ - wanted) <= 3.0f;
                    foundActual |= std::abs(outputRayZ - got) <= 3.0f;
                }
            }
            if(!foundExpected || !foundActual) ++unmatched;
        }
    }
    CAPTURE(mismatches, unmatched, firstX, firstY, firstExpected, firstActual);
    // Different float rounding can change which depth wins an occlusion. Require
    // >=99.99% exact pixels and independently valid 3D projections for both winners.
    REQUIRE(unmatched == 0);
    REQUIRE(mismatches <= (source ? size_t(width) * height / 10000 : 0));
}

struct SyntheticAlignmentFrame {
    std::shared_ptr<dai::ImgFrame> input, output;
};

std::vector<SyntheticAlignmentFrame> runSyntheticBackendSequence(bool runOnHost, dai::Backend backend, bool hardwareRemaps) {
    struct BackendLog {
        std::mutex mutex;
        std::condition_variable changed;
        std::vector<std::string> paths;
    };
    std::vector<SyntheticAlignmentFrame> result;
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<SyntheticAlignmentCase> geometries = {{640, 400, 0, 75, 0}, {1280, 800, 0, -75, 0}};
    // RVC2 hardware remaps are checked against independent projected points for rotations;
    // exact host/device pixel equality here uses unrotated maps on that platform.
    if(!hardwareRemaps) {
        geometries.push_back({320, 240, 10, 75, 50});
        geometries.push_back({320, 240, -7, -75, -30});
    }
    for(const auto& geometry : geometries) {
        auto log = std::make_shared<BackendLog>();
        dai::Pipeline pipeline(!runOnHost);
        if(!runOnHost && !hardwareRemaps) {
            const auto device = pipeline.getDefaultDevice();
            device->setLogLevel(dai::LogLevel::DEBUG);
            device->setLogOutputLevel(dai::LogLevel::WARN);
            device->addLogCallback([log](const dai::LogMessage& message) {
                if(message.payload.find("ImageAlign processing ") == std::string::npos) return;
                std::lock_guard<std::mutex> lock(log->mutex);
                log->paths.push_back(message.payload.substr(message.payload.rfind(' ') + 1));
                log->changed.notify_all();
            });
        }
        auto align = pipeline.create<dai::node::ImageAlign>();
        align->setRunOnHost(runOnHost);
        align->setBackend(backend);
        align->inputConfig.setWaitForMessage(true);
        auto sources = align->input.createInputQueue();
        auto targets = align->inputAlignTo.createInputQueue();
        auto configs = align->inputConfig.createInputQueue();
        auto outputs = align->outputAligned.createOutputQueue();
        pipeline.start();
        std::vector<std::string> expectedPaths;
        const int width = geometry.width, height = geometry.height;
        const float angle = geometry.yaw * 3.14159265358979323846f / 180.0f;
        const std::vector<std::vector<float>> rotation = {{std::cos(angle), 0, std::sin(angle)}, {0, 1, 0}, {-std::sin(angle), 0, std::cos(angle)}};
        const std::array<std::array<float, 3>, 3> k = {{{float(width), 0, (width - 1) / 2.0f}, {0, float(width), (height - 1) / 2.0f}, {0, 0, 1}}};
        const dai::ImgTransformation sourceTransform(
            width,
            height,
            k,
            dai::CameraModel::Perspective,
            {},
            dai::Extrinsics(rotation, {geometry.baseline, 0, geometry.axialTranslation}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER));
        const dai::ImgTransformation targetTransform(width,
                                                     height,
                                                     k,
                                                     dai::CameraModel::Perspective,
                                                     {},
                                                     dai::Extrinsics(identity, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER));
        std::shared_ptr<dai::ImgFrame> contiguous;
        // Return to the same input after each fallback to catch stale stride/plane/type caches.
        for(int frame = 0; frame < 8; ++frame) {
            const bool gray = frame == 5, empty = frame == 7;
            const int padding = frame >= 1 && frame <= 3 ? 8 : 0;
            const int plane = frame == 2 ? 1000 : (gray ? 1500 : 0);
            CAPTURE(runOnHost, backend, width, height, geometry.yaw, geometry.baseline, frame, padding, plane);
            auto source = makeRuntimeTransformationFrame(
                sourceTransform, dai::CameraBoardSocket::CAM_B, gray ? dai::ImgFrame::Type::GRAY8 : dai::ImgFrame::Type::RAW16, result.size());
            source->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(1)));
            source->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::seconds(2)));
            source->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::seconds(3)));
            const int bytesPerPixel = gray ? 1 : 2;
            const int stride = (width + padding) * bytesPerPixel;
            source->setStride(stride);
            std::vector<uint8_t> pixels(height * stride, 0xff);
            for(int y = 0; y < height; ++y) {
                for(int x = 0; x < width; ++x) {
                    uint16_t depth = 300 + (x * 157 + y * 263) % 9701;
                    if(x % 5 == 0 && y % 7 == 0) depth = 0;
                    if(x % 3 == 0 && y % 11 == 0) depth = 65535;
                    if(empty) depth = 0;
                    pixels[y * stride + x * bytesPerPixel] = static_cast<uint8_t>(depth);
                    if(!gray) pixels[y * stride + x * 2 + 1] = static_cast<uint8_t>(depth >> 8);
                }
            }
            source->setData(std::move(pixels));
            const auto target = makeRuntimeTransformationFrame(targetTransform, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::GRAY8, result.size());
            auto config = std::make_shared<dai::ImageAlignConfig>();
            config->staticDepthPlane = plane;
            configs->send(config);
            if(frame == 0) targets->send(target);
            sources->send(source);
            bool timedOut = false;
            const auto output = outputs->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(output != nullptr);
            requireInputMetadata(*output, *source);
            requireAlignedFrameMetadata(*output, *target);
            REQUIRE(output->transformation.isAlignedTo(targetTransform));
            const auto data = output->getData();
            REQUIRE(data.size() >= output->getStride() * height);
            size_t nonzero = 0;
            for(int y = 0; y < height; ++y) {
                const auto row = data.begin() + y * output->getStride();
                nonzero += std::count_if(row, row + width * bytesPerPixel, [](uint8_t value) { return value != 0; });
            }
            if(empty)
                REQUIRE(nonzero == 0);
            else
                REQUIRE(nonzero > size_t(width * height / 4));
            if(frame == 0) contiguous = output;
            if(frame == 1 || frame == 3 || frame == 4 || frame == 6) {
                requireSyntheticPixels(
                    *contiguous, *output, !runOnHost && !hardwareRemaps && backend != dai::Backend::CPU && (frame == 1 || frame == 3) ? source.get() : nullptr);
            }
            result.push_back({source, output});
            const bool gpu = backend != dai::Backend::CPU && !gray && plane == 0 && padding == 0;
            expectedPaths.push_back(gpu ? "gpu" : (backend == dai::Backend::AUTO ? "cpu/dsp" : "cpu"));
        }
        if(!runOnHost && !hardwareRemaps) {
            std::unique_lock<std::mutex> lock(log->mutex);
            REQUIRE(log->changed.wait_for(lock, std::chrono::seconds(5), [&] { return log->paths.size() >= expectedPaths.size(); }));
            REQUIRE(log->paths == expectedPaths);
        }
        REQUIRE(align->getBackend() == backend);
        pipeline.stop();
    }
    return result;
}

void requireSyntheticBackendParity(bool runOnHost) {
    const bool rvc2 = !runOnHost && dai::Device().getPlatform() == dai::Platform::RVC2;
    const auto reference = runSyntheticBackendSequence(true, dai::Backend::CPU, rvc2);
    for(auto backend : {dai::Backend::AUTO, dai::Backend::CPU, dai::Backend::GPU}) {
        if(rvc2 && backend != dai::Backend::AUTO) continue;
        CAPTURE(runOnHost, backend);
        const auto actual = runSyntheticBackendSequence(runOnHost, backend, rvc2);
        REQUIRE(actual.size() == reference.size());
        for(size_t frame = 0; frame < actual.size(); ++frame) {
            CAPTURE(frame);
            REQUIRE(actual[frame].output->getType() == reference[frame].output->getType());
            REQUIRE(actual[frame].output->getWidth() == reference[frame].output->getWidth());
            REQUIRE(actual[frame].output->getHeight() == reference[frame].output->getHeight());
            // Hardware grayscale interpolation is not numerically identical to OpenCV.
            // Metadata and the subsequent return to RAW16 are checked above for RVC2.
            if(rvc2 && actual[frame].output->getType() != dai::ImgFrame::Type::RAW16) continue;
            requireSyntheticPixels(
                *reference[frame].output,
                *actual[frame].output,
                !runOnHost && !rvc2 && frame % 8 != 2 && actual[frame].output->getType() == dai::ImgFrame::Type::RAW16 ? actual[frame].input.get() : nullptr);
        }
    }
}
}  // namespace

TEST_CASE("Test ImageAlign synthetic depth to image host device parity") {
    requireSyntheticBackendParity(false);
}

TEST_CASE("ImageAlign synthetic depth comparison across host backend preferences") {
    SECTION("Backend parity") {
        requireSyntheticBackendParity(true);
    }
    SECTION("Rounded occlusion winners use the output ray") {
        constexpr int width = 320, height = 240, stride = (width + 8) * 2;
        const float angle = 10.0f * 3.14159265358979323846f / 180.0f;
        const float c = std::cos(angle), s = std::sin(angle);
        const std::array<std::array<float, 3>, 3> k = {{{320, 0, 159.5f}, {0, 320, 119.5f}, {0, 0, 1}}};
        const dai::ImgTransformation sourceTransform(
            width,
            height,
            k,
            dai::CameraModel::Perspective,
            {},
            dai::Extrinsics({{c, 0, s}, {0, 1, 0}, {-s, 0, c}}, {75, 0, 50}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER));
        const dai::ImgTransformation targetTransform(
            width,
            height,
            k,
            dai::CameraModel::Perspective,
            {},
            dai::Extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER));
        auto source = makeRuntimeTransformationFrame(sourceTransform, dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Type::RAW16, 0);
        source->setStride(stride);
        std::vector<uint8_t> pixels(height * stride, 0);
        const auto setDepth = [](auto& data, size_t offset, uint16_t depth) {
            data[offset] = static_cast<uint8_t>(depth);
            data[offset + 1] = static_cast<uint8_t>(depth >> 8);
        };
        setDepth(pixels, 40 * stride + 103 * 2, 7589);
        setDepth(pixels, 40 * stride + 104 * 2, 7746);
        source->setData(std::move(pixels));
        auto expected = makeRuntimeTransformationFrame(targetTransform, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::RAW16, 0);
        auto actual = makeRuntimeTransformationFrame(targetTransform, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::RAW16, 0);
        pixels.assign(width * height * 2, 0);
        // These neighboring source samples project within two pixels of (164, 42).
        // Their source-Z planes intersect that output ray at 7750.080 and 7909.108 mm.
        setDepth(pixels, (42 * width + 164) * 2, 7750);
        expected->setData(pixels);
        setDepth(pixels, (42 * width + 164) * 2, 7909);
        actual->setData(std::move(pixels));
        requireSyntheticPixels(*expected, *actual, source.get());
    }
}

TEST_CASE("RVC2 ImageAlign rejects explicit device backend preferences") {
    if(dai::Device().getPlatform() != dai::Platform::RVC2) SKIP("RVC2-only backend rejection");
    for(auto backend : {dai::Backend::CPU, dai::Backend::GPU}) {
        CAPTURE(backend);
        dai::Pipeline pipeline;
        auto align = pipeline.create<dai::node::ImageAlign>();
        align->setBackend(backend);
        align->input.createInputQueue();
        align->inputAlignTo.createInputQueue();
        align->outputAligned.createOutputQueue();
        REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("RVC2 ImageAlign supports only the AUTO backend"));
    }
}

void runImageAlignRotatedCoverageTest(bool runOnHost, bool distorted = false) {
    const std::array<std::array<float, 3>, 3> depthK = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    auto imageK = depthK;
    imageK[0][2] = 64.0f;
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics imageExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation imageTransformation(128, 48, imageK, dai::CameraModel::Perspective, {}, imageExtrinsics);
    const std::vector<float> distortion = distorted ? std::vector<float>{-8.0f / 27.0f, 0, 0, 0, 0} : std::vector<float>{};
    for(const float sign : {-1.0f, 1.0f}) {
        CAPTURE(sign);
        const float angle = sign * 20.0f * 3.14159265f / 180.0f;
        const std::vector<std::vector<float>> rotation = {
            {std::cos(angle), 0.0f, std::sin(angle)}, {0.0f, 1.0f, 0.0f}, {-std::sin(angle), 0.0f, std::cos(angle)}};
        const dai::Extrinsics depthExtrinsics(rotation, {sign * 70.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation depthTransformation(64, 48, depthK, dai::CameraModel::Perspective, distortion, depthExtrinsics);
        const auto aligned = alignSyntheticDepth(runOnHost, depthTransformation, imageTransformation);

        // Project a source point near the outer edge into the target independently of the rectification maps.
        const auto r = depthTransformation.getRotationMatrixTo(imageTransformation);
        const auto t = depthTransformation.getTranslationVectorTo(imageTransformation, false, dai::LengthUnit::MILLIMETER);
        // This known ray lands exactly on an integer source pixel, with or without radial distortion.
        const float rayX = sign * 0.375f;
        const float sourceX = 32.0f + 64.0f * rayX * (1.0f + (distorted ? distortion[0] * rayX * rayX : 0.0f));
        REQUIRE(std::abs(sourceX - std::round(sourceX)) < 1e-4f);
        const float z = 1000.0f + 7.0f * sourceX + 11.0f * 24.0f;
        const float x = rayX * z;
        const float targetX = r[0][0] * x + r[0][2] * z + t[0];
        const float targetZ = r[2][0] * x + r[2][2] * z + t[2];
        const int u = static_cast<int>(std::lround(64.0f * targetX / targetZ + 64.0f));
        REQUIRE(u >= 2);
        REQUIRE(u < 126);
        const auto data = aligned->getData();
        for(int dx = -1; dx <= 1; ++dx) {
            const size_t offset = 2 * (24 * 128 + u + dx);
            const uint16_t depth = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
            REQUIRE(depth > 1000);
        }
    }
}

TEST_CASE("Test ImageAlign preserves rotated depth coverage on host") {
    runImageAlignRotatedCoverageTest(true);
}

TEST_CASE("Test ImageAlign preserves rotated depth coverage on device") {
    runImageAlignRotatedCoverageTest(false);
}

TEST_CASE("Test ImageAlign preserves distorted depth coverage on host") {
    runImageAlignRotatedCoverageTest(true, true);
}

TEST_CASE("Test ImageAlign preserves distorted depth coverage on device") {
    runImageAlignRotatedCoverageTest(false, true);
}

namespace {
void runImageAlignZeroShiftTest(bool runOnHost) {
    const std::array<std::array<float, 3>, 3> k = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics imageExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation imageTransformation(64, 48, k, dai::CameraModel::Perspective, {}, imageExtrinsics);
    std::vector<uint8_t> inputPixels(64 * 48 * 2, 0);
    constexpr uint16_t DEPTH = 2000;
    inputPixels[2 * (24 * 64 + 32)] = static_cast<uint8_t>(DEPTH);
    inputPixels[2 * (24 * 64 + 32) + 1] = static_cast<uint8_t>(DEPTH >> 8);

    for(const auto& [baseline, displacement] : {std::pair<float, int>{1.0f, 0},
                                                {-1.0f, 0},
                                                {15.3125f, 0},
                                                {-15.3125f, 0},
                                                {15.9375f, 1},
                                                {-15.9375f, -1},
                                                {31.25f, 1},
                                                {-31.25f, -1},
                                                {140.0f, 4},
                                                {-140.0f, -4},
                                                {141.25f, 5},
                                                {-141.25f, -5}}) {
        CAPTURE(baseline, displacement);
        const dai::Extrinsics depthExtrinsics(identity, {baseline, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation depthTransformation(64, 48, k, dai::CameraModel::Perspective, {}, depthExtrinsics);
        const auto aligned = alignSyntheticDepth(runOnHost, depthTransformation, imageTransformation, inputPixels);
        const auto data = aligned->getData();
        const int shiftedX = 32 + displacement;
        const int adjacentX = shiftedX + (baseline > 0 ? 1 : -1);
        for(int y = 0; y < 48; ++y) {
            for(int x = 0; x < 64; ++x) {
                const size_t offset = 2 * (y * 64 + x);
                const uint16_t actual = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
                const uint16_t expected = y == 24 && (x == shiftedX || (displacement != 0 && x == adjacentX)) ? DEPTH : 0;
                CAPTURE(x, y);
                CHECK(actual == expected);
            }
        }
    }
}
}  // namespace

TEST_CASE("Test ImageAlign only splats depth for nonzero pixel displacement on host") {
    runImageAlignZeroShiftTest(true);
}

TEST_CASE("Test ImageAlign only splats depth for nonzero pixel displacement on device") {
    runImageAlignZeroShiftTest(false);
}

namespace {
void runImageAlignSignedShiftTest(bool runOnHost) {
    const std::array<std::array<float, 3>, 3> k = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics imageExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation imageTransformation(64, 48, k, dai::CameraModel::Perspective, {}, imageExtrinsics);
    std::vector<uint8_t> inputPixels(64 * 48 * 2, 0);
    constexpr uint16_t DEPTH = 2000;
    inputPixels[2 * (24 * 64 + 32)] = static_cast<uint8_t>(DEPTH);
    inputPixels[2 * (24 * 64 + 32) + 1] = static_cast<uint8_t>(DEPTH >> 8);

    for(const auto& [baseline, displacement] : {std::pair<float, int>{31.25f, 1}, {-31.25f, -1}, {140.0f, 4}, {-140.0f, -4}, {141.25f, 5}, {-141.25f, -5}}) {
        CAPTURE(baseline, displacement);
        const dai::Extrinsics depthExtrinsics(identity, {baseline, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation depthTransformation(64, 48, k, dai::CameraModel::Perspective, {}, depthExtrinsics);
        const auto aligned = alignSyntheticDepth(runOnHost, depthTransformation, imageTransformation, inputPixels);
        const auto data = aligned->getData();
        const int shiftedX = 32 + displacement;
        const int adjacentX = shiftedX + (baseline > 0 ? 1 : -1);
        for(int y = 0; y < 48; ++y) {
            for(int x = 0; x < 64; ++x) {
                const size_t offset = 2 * (y * 64 + x);
                const uint16_t actual = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
                const uint16_t expected = y == 24 && (x == shiftedX || x == adjacentX) ? DEPTH : 0;
                CAPTURE(x, y);
                CHECK(actual == expected);
            }
        }
    }
}
}  // namespace

TEST_CASE("Test ImageAlign rounds signed depth shifts on host") {
    runImageAlignSignedShiftTest(true);
}

TEST_CASE("Test ImageAlign rounds signed depth shifts on device") {
    runImageAlignSignedShiftTest(false);
}

struct UnusableBoundsGeometry {
    float focalLength;
    float yawDegrees;
    std::array<float, 3> translation;
};

// Each geometry makes the expanded rectified bounds unusable: a wide lens with a large rotation, a 90 deg rotation, or a wide lens
// with the baseline turned 45 deg in depth (a broken stereo extrinsic in the EEPROM can turn the baseline like this).
const std::array<UnusableBoundsGeometry, 3> unusableBoundsGeometries = {{
    {20.0f, 40.0f, {70.0f, 0.0f, 0.0f}},
    {64.0f, 90.0f, {70.0f, 0.0f, 0.0f}},
    {20.0f, 0.0f, {50.0f, 0.0f, -50.0f}},
}};

void runImageAlignUnusableBoundsTest(bool runOnHost) {
    for(const auto& geometry : unusableBoundsGeometries) {
        CAPTURE(geometry.focalLength, geometry.yawDegrees, geometry.translation[0], geometry.translation[2]);
        const float f = geometry.focalLength;
        const std::array<std::array<float, 3>, 3> k = {{{f, 0.0f, 32.0f}, {0.0f, f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
        const float angle = geometry.yawDegrees * 3.14159265f / 180.0f;
        const std::vector<std::vector<float>> rotation = {
            {std::cos(angle), 0.0f, std::sin(angle)}, {0.0f, 1.0f, 0.0f}, {-std::sin(angle), 0.0f, std::cos(angle)}};
        const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        const dai::Point3f translation(geometry.translation[0], geometry.translation[1], geometry.translation[2]);
        const dai::Extrinsics depthExtrinsics(rotation, translation, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::Extrinsics imageExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation depthTransformation(64, 48, k, dai::CameraModel::Perspective, {}, depthExtrinsics);
        const dai::ImgTransformation imageTransformation(64, 48, k, dai::CameraModel::Perspective, {}, imageExtrinsics);

        // The node falls back to the input-size canvas and keeps running, instead of stopping the pipeline.
        const auto aligned = alignSyntheticDepth(runOnHost, depthTransformation, imageTransformation);
        REQUIRE(aligned->getWidth() == 64);
        REQUIRE(aligned->getHeight() == 48);
    }
}

TEST_CASE("Test ImageAlign falls back to the input canvas for unusable rectified bounds on host") {
    runImageAlignUnusableBoundsTest(true);
}

TEST_CASE("Test ImageAlign falls back to the input canvas for unusable rectified bounds on device") {
    runImageAlignUnusableBoundsTest(false);
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

    auto requireAligned = [&](const std::shared_ptr<dai::ImgFrame>& aligned, const dai::ImgTransformation& currentAlignToTransformation, int64_t sequenceNum) {
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
    auto sendAndRequireAligned = [&](const dai::ImgTransformation& currentInputTransformation, const dai::ImgTransformation& currentAlignToTransformation) {
        const auto [alignWidth, alignHeight] = currentAlignToTransformation.getSize();
        for(int attempt = 0; attempt < 10; ++attempt) {
            ++sequenceNum;
            CAPTURE(sequenceNum, attempt);
            enqueue(currentInputTransformation, currentAlignToTransformation, sequenceNum);
            auto aligned = outputQueue->get<dai::ImgFrame>();
            REQUIRE(aligned != nullptr);
            REQUIRE(aligned->getSequenceNum() == sequenceNum);
            if(aligned->getWidth() == alignWidth && aligned->getHeight() == alignHeight && aligned->transformation.isAlignedTo(currentAlignToTransformation)) {
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

TEST_CASE("ImageAlign refreshes stride and static-depth caches on host") {
    constexpr int width = 64, height = 48;
    const std::array<std::array<float, 3>, 3> k = {{{64, 0, 32}, {0, 64, 24}, {0, 0, 1}}};
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const dai::ImgTransformation targetTransform(
        width, height, k, dai::CameraModel::Perspective, {}, dai::Extrinsics(identity, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER));
    const dai::ImgTransformation sourceTransform(
        width, height, k, dai::CameraModel::Perspective, {}, dai::Extrinsics(identity, {75, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER));
    dai::Pipeline pipeline(false);
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);
    align->inputConfig.setWaitForMessage(true);
    auto configs = align->inputConfig.createInputQueue();
    auto sources = align->input.createInputQueue();
    auto targets = align->inputAlignTo.createInputQueue();
    auto outputs = align->outputAligned.createOutputQueue();
    pipeline.start();
    targets->send(makeRuntimeTransformationFrame(targetTransform, dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Type::GRAY8, 0));
    std::vector<uint8_t> reference;
    int sequence = 0;
    for(const auto& [padding, plane] : {std::pair<int, int>{0, 0}, {8, 0}, {8, 1000}, {8, 0}, {0, 0}}) {
        CAPTURE(padding, plane);
        auto source = makeRuntimeTransformationFrame(sourceTransform, dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Type::RAW16, sequence++);
        const int stride = (width + padding) * 2;
        std::vector<uint8_t> data(stride * height, 0xff);
        for(int y = 0; y < height; ++y)
            for(int x = 0; x < width; ++x) {
                const uint16_t depth = 800 + (x * 17 + y * 31) % 1000;
                data[y * stride + x * 2] = depth & 0xff;
                data[y * stride + x * 2 + 1] = depth >> 8;
            }
        source->setStride(stride);
        source->setData(std::move(data));
        auto config = std::make_shared<dai::ImageAlignConfig>();
        config->staticDepthPlane = plane;
        configs->send(config);
        sources->send(source);
        auto output = outputs->get<dai::ImgFrame>();
        REQUIRE(output != nullptr);
        REQUIRE(output->getSequenceNum() == sequence - 1);
        const auto pixels = output->getData();
        if(reference.empty())
            reference.assign(pixels.begin(), pixels.end());
        else if(plane == 0)
            REQUIRE(std::equal(reference.begin(), reference.end(), pixels.begin(), pixels.end()));
    }
    pipeline.stop();
}

TEST_CASE("ImageAlign backend preferences serialize on host") {
    dai::Pipeline pipeline(false);
    auto align = pipeline.create<dai::node::ImageAlign>();
    REQUIRE(align->getBackend() == dai::Backend::AUTO);
    align->setRunOnHost(true);
    for(auto backend : {dai::Backend::CPU, dai::Backend::GPU, dai::Backend::AUTO}) {
        REQUIRE(&align->setBackend(backend) == align.get());
        REQUIRE(align->getBackend() == backend);
        REQUIRE(align->runOnHost());
        for(auto format : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
            std::vector<uint8_t> bytes;
            align->properties.serialize(bytes, format);
            dai::ImageAlignProperties restored;
            REQUIRE(dai::utility::deserialize(bytes.data(), bytes.size(), restored, format));
            REQUIRE(restored.backend == backend);
            REQUIRE(restored.numShaves == align->properties.numShaves);
        }
    }
    REQUIRE_THROWS_AS(align->setBackend(dai::Backend::HW), std::invalid_argument);
    REQUIRE_THROWS_AS(align->setBackend(static_cast<dai::Backend>(255)), std::invalid_argument);
    REQUIRE(align->getBackend() == dai::Backend::AUTO);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
TEST_CASE("Fused ImageAlign depth matches staged remaps with padded input") {
    for(const auto size : {cv::Size(96, 64), cv::Size(640, 400)}) {
        cv::Mat storage(size.height, size.width + 8, CV_16UC1);
        auto input = storage.colRange(0, size.width);
        cv::RNG random(42);
        random.fill(input, cv::RNG::UNIFORM, 0, 65536);
        input(cv::Rect(0, 0, 8, 8)).setTo(0);
        input(cv::Rect(8, 0, 8, 8)).setTo(65535);
        const cv::Size rectifiedSize(size.width + 6, size.height + 4), targetSize(size.width / 2, size.height / 2);
        cv::Mat x1(rectifiedSize, CV_32F), y1(rectifiedSize, CV_32F), x2(targetSize, CV_32F), y2(targetSize, CV_32F);
        for(int y = 0; y < x1.rows; ++y)
            for(int x = 0; x < x1.cols; ++x) {
                x1.at<float>(y, x) = x - 2.2f;
                y1.at<float>(y, x) = y + 1.1f;
            }
        for(int y = 0; y < x2.rows; ++y)
            for(int x = 0; x < x2.cols; ++x) {
                x2.at<float>(y, x) = x * 2 + .3f;
                y2.at<float>(y, x) = y * 2 - 1.4f;
            }
        const auto first = dai::detail::nearestDepthOffsets(x1, y1, input.size(), input.step1());
        const auto second = dai::detail::nearestDepthOffsets(x2, y2, rectifiedSize, rectifiedSize.width);
        std::vector<float> rays(x1.total()), scales(x2.total());
        for(size_t i = 0; i < rays.size(); ++i) rays[i] = i % 31 ? 1.0f + (i % x1.cols) * .0001f : 0;
        for(size_t i = 0; i < scales.size(); ++i) scales[i] = i % 47 ? 1.01f : 0;
        for(float shift : {-4700.0f, 0.0f, 4700.0f}) {
            CAPTURE(size, shift);
            std::array<float, 65536> lut{};
            for(size_t i = 1; i < lut.size(); ++i) lut[i] = shift / i;
            cv::Mat rectified, shifted(rectifiedSize, CV_16UC1, cv::Scalar(0)), expected;
            cv::remap(input, rectified, x1, y1, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
            for(int y = 0; y < rectified.rows; ++y)
                for(int n = 0; n < rectified.cols; ++n) {
                    int x = shift > 0 ? rectified.cols - 1 - n : n;
                    uint16_t depth = rectified.at<uint16_t>(y, x);
                    float ray = rays[y * rectified.cols + x];
                    if(depth == 0 || ray == 0) continue;
                    float projected = x + lut[depth] * ray;
                    if(!(projected > -1.5f && projected < rectified.cols + .5f)) continue;
                    int u = projected >= .5f ? int(projected + .5f) : (projected <= -.5f ? -1 : 0);
                    if(u >= 0 && u < rectified.cols) shifted.at<uint16_t>(y, u) = depth;
                    if(u != x) {
                        u += shift > 0 ? 1 : -1;
                        if(u >= 0 && u < rectified.cols) shifted.at<uint16_t>(y, u) = depth;
                    }
                }
            cv::remap(shifted, expected, x2, y2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
            dai::impl::imageAlignConvertDepth(expected.data, expected.cols, expected.rows, expected.step, scales.data(), -30);
            cv::Mat fusedShift, outputStorage(targetSize.height, targetSize.width + 4, CV_16UC1, cv::Scalar(123));
            auto actual = outputStorage.colRange(0, targetSize.width);
            dai::detail::remapAndShiftDepth(input, fusedShift, rectifiedSize, first, lut, shift > 0, rays);
            dai::detail::remapAndConvertDepth(fusedShift, actual, second, scales, -30);
            REQUIRE(cv::countNonZero(fusedShift != shifted) == 0);
            REQUIRE(cv::countNonZero(actual != expected) == 0);
            REQUIRE(cv::countNonZero(outputStorage.colRange(targetSize.width, outputStorage.cols) != 123) == 0);
        }
    }
}

#endif

TEST_CASE("ImageAlign axial depth conversion preserves holes and padded rows") {
    using namespace dai::impl;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    REQUIRE(imageAlignScaleDepth(0, 1.0f, 50.0f) == 0);
    REQUIRE(imageAlignScaleDepth(1000, 1.25f, 50.0f) == 1313);
    REQUIRE(imageAlignScaleDepth(1000, 1.0f, -1100.0f) == 0);
    REQUIRE(imageAlignScaleDepth(65535, 1.0f) == 65535);
    REQUIRE(imageAlignScaleDepth(65535, 2.0f) == 0);
    REQUIRE(imageAlignScaleDepth(1000, 0.0f) == 0);
    REQUIRE(imageAlignScaleDepth(1000, nan) == 0);
    REQUIRE(imageAlignDepthScales(1, 1, {0, 0, 0})[0] == 0);
    REQUIRE(imageAlignDepthScales(1, 1, {0, 0, nan})[0] == 0);

    // Independent plane equation for the reported Ry(10deg), [75,0,50] mm geometry.
    const float angle = 10.0f * 3.14159265358979323846f / 180.0f;
    const float nx = std::sin(angle), nz = std::cos(angle);
    const float offset = nx * 75.0f + nz * 50.0f;
    const std::array<float, 3> row{nx / 250.0f, 0.0f, nz - nx * 159.5f / 250.0f};
    const auto scales = imageAlignDepthScales(320, 240, row);
    const auto rayZ = imageAlignDepthScales(320, 240, row, false);
    for(int y = 0; y < 240; ++y) {
        for(int x = 0; x < 320; ++x) {
            const float expected = (1000.0f + offset) / (nx * (x - 159.5f) / 250.0f + nz);
            const size_t index = y * 320 + x;
            REQUIRE(std::abs(imageAlignScaleDepth(1000, scales[index], offset) - expected) <= 0.501f);
            REQUIRE(std::abs(scales[index] * rayZ[index] - 1.0f) < 1e-6f);
        }
    }

    // In-place conversion must honor row padding and preserve holes despite nonzero translation.
    uint16_t image[] = {1000, 0, 0xaaaa, 2000, 3000, 0xbbbb};
    const float paddedScales[] = {1, 2, 0.5f, 0};
    imageAlignConvertDepth(reinterpret_cast<uint8_t*>(image), 2, 2, 6, paddedScales, 50);
    REQUIRE(image[0] == 1050);
    REQUIRE(image[1] == 0);
    REQUIRE(image[2] == 0xaaaa);
    REQUIRE(image[3] == 1025);
    REQUIRE(image[4] == 0);
    REQUIRE(image[5] == 0xbbbb);
}
