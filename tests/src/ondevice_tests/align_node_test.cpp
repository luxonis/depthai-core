#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <type_traits>
#include <vector>

#include "depthai/depthai.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace {

constexpr size_t kSourceWidth = 320;
constexpr size_t kSourceHeight = 240;
constexpr size_t kTargetWidth = 400;
constexpr size_t kTargetHeight = 300;
constexpr size_t kUpdatedTargetWidth = 480;
constexpr size_t kUpdatedTargetHeight = 360;

array<array<float, 3>, 3> makeIntrinsics(size_t width, size_t height) {
    return {{{150.0F, 0.0F, static_cast<float>(width) / 2.0F}, {0.0F, 148.0F, static_cast<float>(height) / 2.0F}, {0.0F, 0.0F, 1.0F}}};
}

dai::Extrinsics makeExtrinsics(float translationXMillimeters = 0.0F) {
    static const vector<vector<float>> rotation = {{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}};
    return {rotation, {translationXMillimeters, 0.0F, 0.0F}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER};
}

vector<float> makeDistortionCoefficients() {
    return {0.12F, -0.04F, 0.0015F, 0.0005F, 0.01F};
}

dai::ImgTransformation makeTransform(size_t width, size_t height, bool distorted = false) {
    dai::ImgTransformation transform(kSourceWidth,
                                     kSourceHeight,
                                     makeIntrinsics(kSourceWidth, kSourceHeight),
                                     dai::CameraModel::Perspective,
                                     distorted ? makeDistortionCoefficients() : vector<float>{},
                                     makeExtrinsics());
    if(width != kSourceWidth || height != kSourceHeight) {
        transform.addScale(static_cast<float>(width) / static_cast<float>(kSourceWidth), static_cast<float>(height) / static_cast<float>(kSourceHeight));
    }
    return transform;
}

dai::ImgTransformation makeTransformNoExtrinsics(size_t width, size_t height) {
    dai::ImgTransformation transform(kSourceWidth, kSourceHeight, makeIntrinsics(kSourceWidth, kSourceHeight), dai::CameraModel::Perspective, vector<float>{});
    if(width != kSourceWidth || height != kSourceHeight) {
        transform.addScale(static_cast<float>(width) / static_cast<float>(kSourceWidth), static_cast<float>(height) / static_cast<float>(kSourceHeight));
    }
    return transform;
}

dai::ImgTransformation makeTransformWithTranslation(float translationXMillimeters) {
    return {kSourceWidth,
            kSourceHeight,
            makeIntrinsics(kSourceWidth, kSourceHeight),
            dai::CameraModel::Perspective,
            vector<float>{},
            makeExtrinsics(translationXMillimeters)};
}

template <typename MsgT>
constexpr bool expectsUndistortedOutput() {
    return is_same_v<MsgT, dai::ImgFrame> || is_same_v<MsgT, dai::SegmentationMask> || is_same_v<MsgT, dai::ImgDetections>
           || is_same_v<MsgT, dai::SpatialImgDetections>;
}

dai::ImgTransformation expectedAlignedTransform(const dai::ImgTransformation& alignToTransform, bool undistorted = true) {
    dai::ImgTransformation expected = alignToTransform;
    auto distortion = expected.getDistortionCoefficients();
    if(undistorted && !distortion.empty()) {
        expected.setDistortionCoefficients(vector<float>(distortion.size(), 0.0F));
    }
    return expected;
}

steady_clock::time_point makeTimestamp(int millisecondsOffset) {
    return steady_clock::time_point{} + milliseconds(millisecondsOffset);
}

vector<uint8_t> makeFrameData(size_t width, size_t height, uint8_t seed) {
    vector<uint8_t> data(width * height);
    for(size_t y = 0; y < height; ++y) {
        for(size_t x = 0; x < width; ++x) {
            data[y * width + x] = static_cast<uint8_t>((x * 7 + y * 13 + seed) % 251);
        }
    }
    return data;
}

vector<uint8_t> makeMaskData(size_t width, size_t height) {
    vector<uint8_t> mask(width * height, 255);
    for(size_t y = height / 4; y < (3 * height) / 4; ++y) {
        for(size_t x = width / 5; x < (4 * width) / 5; ++x) {
            mask[y * width + x] = (x < width / 2) ? 0 : 1;
        }
    }
    return mask;
}

template <typename MsgT>
void setCommonMetadata(MsgT& msg, int64_t sequenceNum) {
    msg.setSequenceNum(sequenceNum);
    msg.setTimestamp(makeTimestamp(1000 + static_cast<int>(sequenceNum)));
    msg.setTimestampDevice(makeTimestamp(2000 + static_cast<int>(sequenceNum)));
}

template <typename MsgT>
const dai::ImgTransformation& messageTransformation(const MsgT& msg) {
    REQUIRE(msg.transformation.has_value());
    return msg.transformation.value();
}

const dai::ImgTransformation& messageTransformation(const dai::ImgFrame& msg) {
    return msg.transformation;
}

void requireTransformEqual(const dai::ImgTransformation& expected, const dai::ImgTransformation& actual) {
    REQUIRE(actual.getSize() == expected.getSize());
    REQUIRE(actual.getSourceSize() == expected.getSourceSize());
    REQUIRE(actual.getMatrix() == expected.getMatrix());
    REQUIRE(actual.getMatrixInv() == expected.getMatrixInv());
    REQUIRE(actual.getSourceIntrinsicMatrix() == expected.getSourceIntrinsicMatrix());
    REQUIRE(actual.getSourceIntrinsicMatrixInv() == expected.getSourceIntrinsicMatrixInv());
    REQUIRE(actual.getDistortionModel() == expected.getDistortionModel());
    REQUIRE(actual.getDistortionCoefficients() == expected.getDistortionCoefficients());
    REQUIRE(actual.getExtrinsics().isEqualExtrinsics(expected.getExtrinsics()));
}

template <typename MsgT>
void requireCommonMetadata(const MsgT& expected, const MsgT& actual) {
    REQUIRE(actual.getSequenceNum() == expected.getSequenceNum());
    REQUIRE(actual.getTimestamp() == expected.getTimestamp());
    REQUIRE(actual.getTimestampDevice() == expected.getTimestampDevice());
}

template <typename MsgT>
std::shared_ptr<MsgT> getRequiredMessage(const shared_ptr<dai::MessageQueue>& queue) {
    bool timedOut = false;
    auto msg = queue->get<MsgT>(2s, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(msg != nullptr);
    return msg;
}

template <typename MsgT>
std::shared_ptr<MsgT> createSampleMessage(const dai::ImgTransformation& transform, int64_t sequenceNum);

template <>
shared_ptr<dai::ImgFrame> createSampleMessage<dai::ImgFrame>(const dai::ImgTransformation& transform, int64_t sequenceNum) {
    auto msg = make_shared<dai::ImgFrame>();
    const auto [sourceWidth, sourceHeight] = transform.getSourceSize();
    const auto [width, height] = transform.getSize();

    msg->setSourceSize(static_cast<unsigned int>(sourceWidth), static_cast<unsigned int>(sourceHeight));
    msg->setSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    msg->setType(dai::ImgFrame::Type::GRAY8);
    msg->setData(makeFrameData(width, height, static_cast<uint8_t>(sequenceNum)));
    msg->setInstanceNum(static_cast<unsigned int>(700 + sequenceNum));
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);

    REQUIRE(msg->validateTransformations());
    return msg;
}

template <>
shared_ptr<dai::ImgDetections> createSampleMessage<dai::ImgDetections>(const dai::ImgTransformation& transform, int64_t sequenceNum) {
    auto msg = make_shared<dai::ImgDetections>();
    const auto [width, height] = transform.getSize();

    dai::ImgDetection detection;
    detection.label = 2;
    detection.labelName = "person";
    detection.confidence = 0.83F;
    detection.setBoundingBox(dai::RotatedRect(dai::Rect(8.0F, 6.0F, 20.0F, 14.0F, false), 11.0F));

    msg->detections.push_back(detection);
    msg->setSegmentationMask(makeMaskData(width, height), width, height);
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);
    return msg;
}

template <>
shared_ptr<dai::SpatialImgDetections> createSampleMessage<dai::SpatialImgDetections>(const dai::ImgTransformation& transform, int64_t sequenceNum) {
    auto msg = make_shared<dai::SpatialImgDetections>();
    const auto [width, height] = transform.getSize();

    dai::SpatialImgDetection detection;
    detection.label = 5;
    detection.labelName = "crate";
    detection.confidence = 0.76F;
    detection.setBoundingBox(dai::RotatedRect(dai::Rect(10.0F, 7.0F, 18.0F, 12.0F, false), -6.0F));
    detection.spatialCoordinates = {30.0F, -20.0F, 1200.0F};
    detection.boundingBoxMapping.roi = dai::Rect(0.2F, 0.15F, 0.3F, 0.25F, true);
    detection.boundingBoxMapping.depthThresholds.lowerThreshold = 100;
    detection.boundingBoxMapping.depthThresholds.upperThreshold = 4000;

    msg->detections.push_back(detection);
    msg->unit = dai::LengthUnit::MILLIMETER;
    msg->setSegmentationMask(makeMaskData(width, height), width, height);
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);
    return msg;
}

template <>
shared_ptr<dai::SegmentationMask> createSampleMessage<dai::SegmentationMask>(const dai::ImgTransformation& transform, int64_t sequenceNum) {
    auto msg = make_shared<dai::SegmentationMask>();
    const auto [width, height] = transform.getSize();

    msg->setMask(makeMaskData(width, height), width, height);
    msg->setLabels({"background", "left", "right"});
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);
    return msg;
}

template <>
shared_ptr<dai::AprilTags> createSampleMessage<dai::AprilTags>(const dai::ImgTransformation& transform, int64_t sequenceNum) {
    auto msg = make_shared<dai::AprilTags>();

    dai::AprilTag tag;
    tag.id = 42;
    tag.hamming = 1;
    tag.decisionMargin = 73.5F;
    tag.topLeft = {9.0F, 7.0F, false};
    tag.topRight = {25.0F, 7.0F, false};
    tag.bottomRight = {25.0F, 23.0F, false};
    tag.bottomLeft = {9.0F, 23.0F, false};

    msg->aprilTags.push_back(tag);
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);
    return msg;
}

template <>
shared_ptr<dai::Tracklets> createSampleMessage<dai::Tracklets>(const dai::ImgTransformation& transform, int64_t sequenceNum) {
    auto msg = make_shared<dai::Tracklets>();

    dai::Tracklet tracklet;
    tracklet.id = 17;
    tracklet.label = 2;
    tracklet.age = 4;
    tracklet.status = dai::Tracklet::TrackingStatus::TRACKED;
    tracklet.roi = dai::Rect(7.0F, 6.0F, 22.0F, 16.0F, false);
    tracklet.srcImgDetection.label = 2;
    tracklet.srcImgDetection.labelName = "tracked";
    tracklet.srcImgDetection.confidence = 0.91F;
    tracklet.srcImgDetection.setBoundingBox(dai::RotatedRect(tracklet.roi, 0.0F));
    tracklet.spatialCoordinates = {40.0F, 10.0F, 1400.0F};
    tracklet.velocity = dai::Point3f(1.0F, 0.0F, 0.25F);
    tracklet.speed = 1.03F;

    msg->tracklets.push_back(tracklet);
    msg->unit = dai::LengthUnit::MILLIMETER;
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);
    return msg;
}

vector<uint8_t> makeYuvFrameData(size_t width, size_t height, uint8_t seed) {
    vector<uint8_t> data = makeFrameData(width, height, seed);
    data.resize(width * height * 3 / 2, 128);  // neutral chroma
    return data;
}

// A YUV frame with an optional padded frame buffer layout, the way a camera delivers it.
shared_ptr<dai::ImgFrame> createYuvSampleMessage(
    const dai::ImgTransformation& transform, dai::ImgFrame::Type type, int64_t sequenceNum, size_t stride = 0, size_t planeHeight = 0) {
    auto msg = make_shared<dai::ImgFrame>();
    const auto [sourceWidth, sourceHeight] = transform.getSourceSize();
    const auto [width, height] = transform.getSize();
    if(stride == 0) stride = width;
    if(planeHeight == 0) planeHeight = height;

    msg->setSourceSize(static_cast<unsigned int>(sourceWidth), static_cast<unsigned int>(sourceHeight));
    msg->setSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    msg->setType(type);
    msg->setStride(static_cast<unsigned int>(stride));
    msg->fb.p1Offset = 0;
    msg->fb.p2Offset = static_cast<unsigned int>(stride * planeHeight);
    msg->fb.p3Offset = static_cast<unsigned int>(stride * planeHeight * 5 / 4);
    msg->setData(makeYuvFrameData(stride, planeHeight, static_cast<uint8_t>(sequenceNum)));
    msg->setInstanceNum(static_cast<unsigned int>(700 + sequenceNum));
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);
    return msg;
}

template <typename InputT>
struct AlignmentResult {
    shared_ptr<InputT> aligned;
    shared_ptr<InputT> passthrough;
};

template <typename InputT, typename AlignToT>
AlignmentResult<InputT> runGenericAlignmentOnce(const shared_ptr<InputT>& inputMsg,
                                                const shared_ptr<AlignToT>& alignToMsg,
                                                const std::function<void(dai::node::Align&)>& configure = {}) {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::Align>();
    if(configure) {
        configure(*align);
    }

    auto inputQueue = align->input.createInputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto alignedQueue = align->outputAligned.createOutputQueue();
    auto passthroughQueue = align->passthroughInput.createOutputQueue();

    pipeline.start();
    inputQueue->send(inputMsg);
    alignToQueue->send(alignToMsg);

    AlignmentResult<InputT> result;
    result.aligned = getRequiredMessage<InputT>(alignedQueue);
    result.passthrough = getRequiredMessage<InputT>(passthroughQueue);

    pipeline.stop();
    pipeline.wait();
    return result;
}

void requireMessageMetadata(const dai::ImgFrame& expectedInput, const dai::ImgFrame& actual, const dai::ImgTransformation& expectedTransform) {
    const auto [expectedWidth, expectedHeight] = expectedTransform.getSize();
    requireCommonMetadata(expectedInput, actual);
    requireTransformEqual(expectedTransform, actual.transformation);
    REQUIRE(actual.getInstanceNum() == expectedInput.getInstanceNum());
    REQUIRE(actual.getType() == expectedInput.getType());
    REQUIRE(actual.getWidth() == expectedWidth);
    REQUIRE(actual.getHeight() == expectedHeight);
    REQUIRE(actual.getData().size() == expectedWidth * expectedHeight);
}

void requireMessageMetadata(const dai::ImgDetections& expectedInput, const dai::ImgDetections& actual, const dai::ImgTransformation& expectedTransform) {
    const auto [expectedWidth, expectedHeight] = expectedTransform.getSize();
    requireCommonMetadata(expectedInput, actual);
    requireTransformEqual(expectedTransform, messageTransformation(actual));
    REQUIRE(actual.detections.size() == expectedInput.detections.size());
    REQUIRE(actual.getSegmentationMaskWidth() == expectedWidth);
    REQUIRE(actual.getSegmentationMaskHeight() == expectedHeight);
    REQUIRE(actual.getMaskData().has_value());
    REQUIRE(actual.getMaskData()->size() == expectedWidth * expectedHeight);
}

void requireMessageMetadata(const dai::SpatialImgDetections& expectedInput,
                            const dai::SpatialImgDetections& actual,
                            const dai::ImgTransformation& expectedTransform) {
    const auto [expectedWidth, expectedHeight] = expectedTransform.getSize();
    requireCommonMetadata(expectedInput, actual);
    requireTransformEqual(expectedTransform, messageTransformation(actual));
    REQUIRE(actual.unit == expectedInput.unit);
    REQUIRE(actual.detections.size() == expectedInput.detections.size());
    REQUIRE(actual.getSegmentationMaskWidth() == expectedWidth);
    REQUIRE(actual.getSegmentationMaskHeight() == expectedHeight);
    REQUIRE(actual.getMaskData().has_value());
    REQUIRE(actual.getMaskData()->size() == expectedWidth * expectedHeight);
}

void requireMessageMetadata(const dai::SegmentationMask& expectedInput, const dai::SegmentationMask& actual, const dai::ImgTransformation& expectedTransform) {
    const auto [expectedWidth, expectedHeight] = expectedTransform.getSize();
    requireCommonMetadata(expectedInput, actual);
    requireTransformEqual(expectedTransform, messageTransformation(actual));
    REQUIRE(actual.getLabels() == expectedInput.getLabels());
    REQUIRE(actual.getWidth() == expectedWidth);
    REQUIRE(actual.getHeight() == expectedHeight);
    REQUIRE(actual.getMaskData().size() == expectedWidth * expectedHeight);
}

void requireMessageMetadata(const dai::AprilTags& expectedInput, const dai::AprilTags& actual, const dai::ImgTransformation& expectedTransform) {
    requireCommonMetadata(expectedInput, actual);
    requireTransformEqual(expectedTransform, messageTransformation(actual));
    REQUIRE(actual.aprilTags.size() == expectedInput.aprilTags.size());
}

void requireMessageMetadata(const dai::Tracklets& expectedInput, const dai::Tracklets& actual, const dai::ImgTransformation& expectedTransform) {
    requireCommonMetadata(expectedInput, actual);
    requireTransformEqual(expectedTransform, messageTransformation(actual));
    REQUIRE(actual.unit == expectedInput.unit);
    REQUIRE(actual.tracklets.size() == expectedInput.tracklets.size());
}

shared_ptr<dai::ImgFrame> createUniformDepthFrame(const dai::ImgTransformation& transform, uint16_t depthMillimeters, int64_t sequenceNum) {
    auto msg = make_shared<dai::ImgFrame>();
    const auto [sourceWidth, sourceHeight] = transform.getSourceSize();
    const auto [width, height] = transform.getSize();

    msg->setSourceSize(static_cast<unsigned int>(sourceWidth), static_cast<unsigned int>(sourceHeight));
    msg->setSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    msg->setType(dai::ImgFrame::Type::RAW16);
    vector<uint8_t> data(width * height * sizeof(uint16_t));
    auto* pixels = reinterpret_cast<uint16_t*>(data.data());
    fill(pixels, pixels + width * height, depthMillimeters);
    msg->setData(std::move(data));
    msg->setInstanceNum(static_cast<unsigned int>(700 + sequenceNum));
    setCommonMetadata(*msg, sequenceNum);
    msg->setTransformation(transform);

    REQUIRE(msg->validateTransformations());
    return msg;
}

const uint16_t* depthRow(const dai::ImgFrame& frame, size_t row) {
    return reinterpret_cast<const uint16_t*>(frame.getData().data()) + row * frame.getWidth();
}

// Custom transformable message, mirroring what Python users implement by subclassing dai.TransformableBuffer.
class TestLineMessage : public dai::TransformableBuffer {
   public:
    dai::Point2f startPoint;
    dai::Point2f endPoint;

    std::shared_ptr<dai::TransformableBuffer> transformTo(const dai::ImgTransformation& target) const override {
        auto source = getTransformation();
        if(!source.has_value()) {
            throw std::runtime_error("Source transformation is not set");
        }
        auto out = std::make_shared<TestLineMessage>();
        out->startPoint = source->remapPointTo(target, startPoint);
        out->endPoint = source->remapPointTo(target, endPoint);
        out->setTransformation(target);
        return out;
    }
};

template <typename InputT, typename AlignToT>
void runGenericMetadataCase() {
    const auto inputTransform = makeTransform(kSourceWidth, kSourceHeight, false);
    const auto alignToTransform = makeTransform(kTargetWidth, kTargetHeight, true);

    auto inputMsg = createSampleMessage<InputT>(inputTransform, 11);
    auto alignToMsg = createSampleMessage<AlignToT>(alignToTransform, 27);
    auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

    requireMessageMetadata(*inputMsg, *result.aligned, expectedAlignedTransform(alignToTransform, expectsUndistortedOutput<InputT>()));
    requireMessageMetadata(*inputMsg, *result.passthrough, inputTransform);
}

template <typename AlignToT>
void runCameraInputMetadataCase() {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::Align>();
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* cameraOutput = camera->requestOutput({kSourceWidth, kSourceHeight}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH, std::nullopt, false);
    REQUIRE(cameraOutput != nullptr);
    cameraOutput->link(align->input);

    auto cameraQueue = cameraOutput->createOutputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto alignedQueue = align->outputAligned.createOutputQueue();
    auto passthroughQueue = align->passthroughInput.createOutputQueue();

    pipeline.start();

    // Build the alignTo transformation from a real camera frame, so its extrinsics resolve against the input.
    auto cameraFrame = cameraQueue->get<dai::ImgFrame>();
    REQUIRE(cameraFrame != nullptr);
    dai::ImgTransformation alignToTransform = cameraFrame->transformation;
    alignToTransform.addScale(static_cast<float>(kTargetWidth) / static_cast<float>(cameraFrame->getWidth()),
                              static_cast<float>(kTargetHeight) / static_cast<float>(cameraFrame->getHeight()));
    alignToQueue->send(createSampleMessage<AlignToT>(alignToTransform, 27));

    auto aligned = alignedQueue->get<dai::ImgFrame>();
    auto passthrough = passthroughQueue->get<dai::ImgFrame>();
    REQUIRE(aligned != nullptr);
    REQUIRE(passthrough != nullptr);

    // The point of this case: a camera frame reaches the passthrough with its buffer intact. A camera frame is
    // stride aligned on the device, so the buffer follows the declared layout rather than width * height.
    REQUIRE(passthrough->getType() == dai::ImgFrame::Type::NV12);
    REQUIRE(passthrough->getWidth() == cameraFrame->getWidth());
    REQUIRE(passthrough->getHeight() == cameraFrame->getHeight());
    REQUIRE(passthrough->getStride() >= passthrough->getWidth());
    REQUIRE(passthrough->getPlaneHeight() >= passthrough->getHeight());
    REQUIRE(passthrough->getData().size() == passthrough->getStride() * passthrough->getPlaneHeight() * 3 / 2);
    requireTransformEqual(cameraFrame->transformation, passthrough->transformation);

    // The aligned frame follows the alignTo geometry and drops the distortion, being pixel data.
    REQUIRE(aligned->getWidth() == kTargetWidth);
    REQUIRE(aligned->getHeight() == kTargetHeight);
    REQUIRE(aligned->getData().size() == kTargetWidth * kTargetHeight * 3 / 2);
    requireTransformEqual(expectedAlignedTransform(alignToTransform), aligned->transformation);

    // Both outputs describe the same input frame.
    REQUIRE(aligned->getSequenceNum() == passthrough->getSequenceNum());
    REQUIRE(aligned->getTimestamp() == passthrough->getTimestamp());
    REQUIRE(aligned->getInstanceNum() == passthrough->getInstanceNum());

    pipeline.stop();
    pipeline.wait();
}
}  // namespace

TEST_CASE("Test Align generic path message to ImgFrame metadata") {
    SECTION("ImgDetections -> ImgFrame") {
        runGenericMetadataCase<dai::ImgDetections, dai::ImgFrame>();
    }
    SECTION("SpatialImgDetections -> ImgFrame") {
        runGenericMetadataCase<dai::SpatialImgDetections, dai::ImgFrame>();
    }
    SECTION("SegmentationMask -> ImgFrame") {
        runGenericMetadataCase<dai::SegmentationMask, dai::ImgFrame>();
    }
    SECTION("AprilTags -> ImgFrame") {
        runGenericMetadataCase<dai::AprilTags, dai::ImgFrame>();
    }
    SECTION("Tracklets -> ImgFrame") {
        runGenericMetadataCase<dai::Tracklets, dai::ImgFrame>();
    }
}

TEST_CASE("Test Align generic path ImgFrame to message metadata") {
    SECTION("ImgFrame -> ImgDetections") {
        runCameraInputMetadataCase<dai::ImgDetections>();
    }
    SECTION("ImgFrame -> SpatialImgDetections") {
        runCameraInputMetadataCase<dai::SpatialImgDetections>();
    }
    SECTION("ImgFrame -> SegmentationMask") {
        runCameraInputMetadataCase<dai::SegmentationMask>();
    }
    SECTION("ImgFrame -> AprilTags") {
        runCameraInputMetadataCase<dai::AprilTags>();
    }
    SECTION("ImgFrame -> Tracklets") {
        runCameraInputMetadataCase<dai::Tracklets>();
    }
}

TEST_CASE("Test Align generic path message to message metadata") {
    SECTION("ImgDetections -> ImgDetections") {
        runGenericMetadataCase<dai::ImgDetections, dai::ImgDetections>();
    }
    SECTION("SpatialImgDetections -> SpatialImgDetections") {
        runGenericMetadataCase<dai::SpatialImgDetections, dai::SpatialImgDetections>();
    }
    SECTION("SegmentationMask -> SegmentationMask") {
        runGenericMetadataCase<dai::SegmentationMask, dai::SegmentationMask>();
    }
    SECTION("AprilTags -> AprilTags") {
        runGenericMetadataCase<dai::AprilTags, dai::AprilTags>();
    }
    SECTION("Tracklets -> Tracklets") {
        runGenericMetadataCase<dai::Tracklets, dai::Tracklets>();
    }
}

TEST_CASE("Test Align keeps the alignTo distortion for detections without a segmentation mask") {
    const auto inputTransform = makeTransform(kSourceWidth, kSourceHeight, false);
    const auto alignToTransform = makeTransform(kTargetWidth, kTargetHeight, true);
    auto alignToMsg = createSampleMessage<dai::ImgFrame>(alignToTransform, 27);

    auto inputMsg = make_shared<dai::ImgDetections>();
    dai::ImgDetection detection;
    detection.label = 2;
    detection.labelName = "person";
    detection.confidence = 0.83F;
    detection.setBoundingBox(dai::RotatedRect(dai::Rect(8.0F, 6.0F, 20.0F, 14.0F, false), 11.0F));
    inputMsg->detections.push_back(detection);
    setCommonMetadata(*inputMsg, 11);
    inputMsg->setTransformation(inputTransform);

    auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

    REQUIRE_FALSE(result.aligned->getMaskData().has_value());
    REQUIRE(result.aligned->detections.size() == 1);
    requireTransformEqual(alignToTransform, messageTransformation(*result.aligned));
}

TEST_CASE("Test Align generic path refreshes rectification metadata when ImgTransformation changes") {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::Align>();

    auto inputQueue = align->input.createInputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto alignedQueue = align->outputAligned.createOutputQueue();

    const auto inputTransform = makeTransform(kSourceWidth, kSourceHeight, false);
    const auto firstAlignToTransform = makeTransform(kTargetWidth, kTargetHeight, true);
    const auto secondAlignToTransform = makeTransform(kUpdatedTargetWidth, kUpdatedTargetHeight, true);

    pipeline.start();

    inputQueue->send(createSampleMessage<dai::SegmentationMask>(inputTransform, 101));
    alignToQueue->send(createSampleMessage<dai::ImgFrame>(firstAlignToTransform, 201));

    auto firstAligned = getRequiredMessage<dai::SegmentationMask>(alignedQueue);
    requireMessageMetadata(*createSampleMessage<dai::SegmentationMask>(inputTransform, 101), *firstAligned, expectedAlignedTransform(firstAlignToTransform));
    REQUIRE(firstAligned->getWidth() == kTargetWidth);
    REQUIRE(firstAligned->getHeight() == kTargetHeight);

    inputQueue->send(createSampleMessage<dai::SegmentationMask>(inputTransform, 102));
    alignToQueue->send(createSampleMessage<dai::ImgFrame>(secondAlignToTransform, 202));

    auto secondAligned = getRequiredMessage<dai::SegmentationMask>(alignedQueue);
    requireMessageMetadata(*createSampleMessage<dai::SegmentationMask>(inputTransform, 102), *secondAligned, expectedAlignedTransform(secondAlignToTransform));
    REQUIRE(secondAligned->getWidth() == kUpdatedTargetWidth);
    REQUIRE(secondAligned->getHeight() == kUpdatedTargetHeight);
    REQUIRE(secondAligned->getMaskData().size() == kUpdatedTargetWidth * kUpdatedTargetHeight);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("Test Align follows a new inputAlignTo message sent at runtime") {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::Align>();

    auto inputQueue = align->input.createInputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto alignedQueue = align->outputAligned.createOutputQueue();

    const auto inputTransform = makeTransform(kSourceWidth, kSourceHeight);
    const auto firstAlignToTransform = makeTransform(kTargetWidth, kTargetHeight);
    const auto secondAlignToTransform = makeTransform(kUpdatedTargetWidth, kUpdatedTargetHeight);

    pipeline.start();

    inputQueue->send(createSampleMessage<dai::ImgFrame>(inputTransform, 301));
    alignToQueue->send(createSampleMessage<dai::ImgFrame>(firstAlignToTransform, 401));

    auto firstAligned = getRequiredMessage<dai::ImgFrame>(alignedQueue);
    REQUIRE(firstAligned->getWidth() == kTargetWidth);
    REQUIRE(firstAligned->getHeight() == kTargetHeight);
    REQUIRE(firstAligned->getData().size() == kTargetWidth * kTargetHeight);
    requireTransformEqual(expectedAlignedTransform(firstAlignToTransform), firstAligned->transformation);

    REQUIRE(alignToQueue->trySend(createSampleMessage<dai::ImgFrame>(secondAlignToTransform, 402)));
    inputQueue->send(createSampleMessage<dai::ImgFrame>(inputTransform, 302));

    auto secondAligned = getRequiredMessage<dai::ImgFrame>(alignedQueue);
    REQUIRE(secondAligned->getSequenceNum() == 302);
    REQUIRE(secondAligned->getWidth() == kUpdatedTargetWidth);
    REQUIRE(secondAligned->getHeight() == kUpdatedTargetHeight);
    REQUIRE(secondAligned->getData().size() == kUpdatedTargetWidth * kUpdatedTargetHeight);
    requireTransformEqual(expectedAlignedTransform(secondAlignToTransform), secondAligned->transformation);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("Test Align aligns metadata messages without calibrated extrinsics") {
    const auto inputTransform = makeTransformNoExtrinsics(kSourceWidth, kSourceHeight);
    const auto alignToTransform = makeTransformNoExtrinsics(kTargetWidth, kTargetHeight);

    SECTION("AprilTags -> ImgFrame") {
        auto inputMsg = createSampleMessage<dai::AprilTags>(inputTransform, 11);
        auto alignToMsg = createSampleMessage<dai::ImgFrame>(alignToTransform, 27);
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

        requireMessageMetadata(*inputMsg, *result.aligned, expectedAlignedTransform(alignToTransform));
        requireMessageMetadata(*inputMsg, *result.passthrough, inputTransform);
    }
}

TEST_CASE("Test Align transforms custom TransformableBuffer messages") {
    const auto inputTransform = makeTransformNoExtrinsics(kSourceWidth, kSourceHeight);
    const auto alignToTransform = makeTransformNoExtrinsics(kTargetWidth, kTargetHeight);

    auto line = make_shared<TestLineMessage>();
    line->setTransformation(inputTransform);
    line->startPoint = dai::Point2f(10.0F, 12.0F);
    line->endPoint = dai::Point2f(40.0F, 30.0F);

    auto alignToMsg = createSampleMessage<dai::ImgFrame>(alignToTransform, 27);
    auto result = runGenericAlignmentOnce(line, alignToMsg, [](dai::node::Align& align) { align.setRunOnHost(true); });

    REQUIRE(result.aligned != nullptr);
    const auto expectedStart = inputTransform.remapPointTo(alignToTransform, line->startPoint);
    const auto expectedEnd = inputTransform.remapPointTo(alignToTransform, line->endPoint);
    REQUIRE(result.aligned->startPoint.x == expectedStart.x);
    REQUIRE(result.aligned->startPoint.y == expectedStart.y);
    REQUIRE(result.aligned->endPoint.x == expectedEnd.x);
    REQUIRE(result.aligned->endPoint.y == expectedEnd.y);
    REQUIRE(result.aligned->getTransformation().has_value());
    requireTransformEqual(alignToTransform, *result.aligned->getTransformation());
}

TEST_CASE("Test Align pixel content") {
    SECTION("identical geometry keeps GRAY8 pixels identical") {
        const auto transform = makeTransformWithTranslation(0.0F);
        auto inputMsg = createSampleMessage<dai::ImgFrame>(transform, 11);
        auto alignToMsg = createSampleMessage<dai::ImgFrame>(transform, 27);
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

        const auto inputData = inputMsg->getData();
        const auto alignedData = result.aligned->getData();
        REQUIRE(alignedData.size() == inputData.size());
        REQUIRE(std::equal(alignedData.begin(), alignedData.end(), inputData.begin()));
    }

    SECTION("pure horizontal baseline keeps GRAY8 pixels in place") {
        const auto inputTransform = makeTransformWithTranslation(75.0F);
        const auto alignToTransform = makeTransformWithTranslation(0.0F);
        auto inputMsg = createSampleMessage<dai::ImgFrame>(inputTransform, 11);
        auto alignToMsg = createSampleMessage<dai::ImgFrame>(alignToTransform, 27);
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

        const auto inputData = inputMsg->getData();
        const auto alignedData = result.aligned->getData();
        REQUIRE(alignedData.size() == inputData.size());
        REQUIRE(std::equal(alignedData.begin(), alignedData.end(), inputData.begin()));
    }
}

TEST_CASE("Test Align rebuilds the frame buffer layout of 1.5 bytes per pixel frames") {
    const auto inputTransform = makeTransform(kSourceWidth, kSourceHeight);
    const auto alignToTransform = makeTransform(kTargetWidth, kTargetHeight);
    auto alignToMsg = createSampleMessage<dai::ImgFrame>(alignToTransform, 27);

    const auto type = GENERATE(dai::ImgFrame::Type::NV12, dai::ImgFrame::Type::YUV420p);

    const auto requireTightlyPackedOutput = [&](const dai::ImgFrame& aligned) {
        REQUIRE(aligned.getType() == type);
        REQUIRE(aligned.getWidth() == kTargetWidth);
        REQUIRE(aligned.getHeight() == kTargetHeight);
        REQUIRE(aligned.getStride() == kTargetWidth);
        REQUIRE(aligned.getPlaneHeight() == kTargetHeight);
        REQUIRE(aligned.getData().size() == kTargetWidth * kTargetHeight * 3 / 2);
    };

    SECTION("tightly packed input") {
        auto inputMsg = createYuvSampleMessage(inputTransform, type, 11);
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

        requireTightlyPackedOutput(*result.aligned);
        requireCommonMetadata(*inputMsg, *result.aligned);
        REQUIRE(result.aligned->getInstanceNum() == inputMsg->getInstanceNum());
        REQUIRE_NOTHROW(result.aligned->getCvFrame());
    }

    SECTION("padded input keeps the padding out of the output") {
        auto inputMsg = createYuvSampleMessage(inputTransform, type, 11, kSourceWidth + 32, kSourceHeight + 16);
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

        requireTightlyPackedOutput(*result.aligned);
        REQUIRE_NOTHROW(result.aligned->getCvFrame());
    }
}

TEST_CASE("Test Align resizes its buffers when the alignTo size or the input type changes") {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::Align>();

    auto inputQueue = align->input.createInputQueue();
    auto alignToQueue = align->inputAlignTo.createInputQueue();
    auto alignedQueue = align->outputAligned.createOutputQueue();

    pipeline.start();

    const auto sendPair = [&](const shared_ptr<dai::ImgFrame>& inputMsg, size_t alignWidth, size_t alignHeight) {
        inputQueue->send(inputMsg);
        alignToQueue->send(createSampleMessage<dai::ImgFrame>(makeTransform(alignWidth, alignHeight), 27));
        return getRequiredMessage<dai::ImgFrame>(alignedQueue);
    };

    const auto inputTransform = makeTransform(kSourceWidth, kSourceHeight);
    const auto nv12Input = [&] { return createYuvSampleMessage(inputTransform, dai::ImgFrame::Type::NV12, 11); };

    auto first = sendPair(nv12Input(), kTargetWidth, kTargetHeight);
    REQUIRE(first->getData().size() == kTargetWidth * kTargetHeight * 3 / 2);

    // Same frame type, larger alignTo. The rectification maps and every buffer follow the new size.
    auto second = sendPair(nv12Input(), kUpdatedTargetWidth, kUpdatedTargetHeight);
    REQUIRE(second->getWidth() == kUpdatedTargetWidth);
    REQUIRE(second->getHeight() == kUpdatedTargetHeight);
    REQUIRE(second->getStride() == kUpdatedTargetWidth);
    REQUIRE(second->getPlaneHeight() == kUpdatedTargetHeight);
    REQUIRE(second->getData().size() == kUpdatedTargetWidth * kUpdatedTargetHeight * 3 / 2);
    REQUIRE_NOTHROW(second->getCvFrame());

    // Same alignTo, an input type with a different byte size.
    auto third = sendPair(createSampleMessage<dai::ImgFrame>(inputTransform, 11), kUpdatedTargetWidth, kUpdatedTargetHeight);
    REQUIRE(third->getType() == dai::ImgFrame::Type::GRAY8);
    REQUIRE(third->getStride() == kUpdatedTargetWidth);
    REQUIRE(third->getData().size() == kUpdatedTargetWidth * kUpdatedTargetHeight);

    // Back to the first alignTo size.
    auto fourth = sendPair(nv12Input(), kTargetWidth, kTargetHeight);
    REQUIRE(fourth->getData().size() == kTargetWidth * kTargetHeight * 3 / 2);
    REQUIRE_NOTHROW(fourth->getCvFrame());

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("Test Align shifts RAW16 depth by disparity") {
    const auto inputTransform = makeTransformWithTranslation(75.0F);
    const auto alignToTransform = makeTransformWithTranslation(0.0F);
    constexpr uint16_t kDepthMillimeters = 1000;
    // Expected disparity = translationX * fx / depth = 75 * 150 / 1000 mm -> 11 pixels
    constexpr int kExpectedShift = 11;

    auto inputMsg = createUniformDepthFrame(inputTransform, kDepthMillimeters, 11);
    auto alignToMsg = createSampleMessage<dai::ImgFrame>(alignToTransform, 27);

    SECTION("per-pixel depth shift") {
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg);

        const auto* row = depthRow(*result.aligned, kSourceHeight / 2);
        REQUIRE(row[2] == 0);
        REQUIRE(row[kExpectedShift - 2] == 0);
        REQUIRE(row[kExpectedShift + 2] == kDepthMillimeters);
        REQUIRE(row[kSourceWidth / 2] == kDepthMillimeters);

        const auto* passthroughRow = depthRow(*result.passthrough, kSourceHeight / 2);
        REQUIRE(passthroughRow[2] == kDepthMillimeters);
    }

    SECTION("static depth plane shift") {
        auto result = runGenericAlignmentOnce(inputMsg, alignToMsg, [](dai::node::Align& align) { align.initialConfig->staticDepthPlane = kDepthMillimeters; });

        const auto* row = depthRow(*result.aligned, kSourceHeight / 2);
        REQUIRE(row[2] == 0);
        REQUIRE(row[kExpectedShift - 2] == 0);
        REQUIRE(row[kExpectedShift + 2] == kDepthMillimeters);
        REQUIRE(row[kSourceWidth / 2] == kDepthMillimeters);
    }
}
