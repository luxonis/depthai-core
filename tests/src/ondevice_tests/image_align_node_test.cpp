#include <catch2/catch_all.hpp>
#include <algorithm>
#include <cstring>

#include "depthai/depthai.hpp"

namespace {

constexpr unsigned kDepthWidth = 64;
constexpr unsigned kDepthHeight = 48;
constexpr float kFocalLength = 100.0f;
constexpr float kPrincipalPointX = static_cast<float>(kDepthWidth) / 2.0f;
constexpr float kPrincipalPointY = static_cast<float>(kDepthHeight) / 2.0f;

std::array<std::array<float, 3>, 3> makeIntrinsics() {
    return {{{kFocalLength, 0.0f, kPrincipalPointX}, {0.0f, kFocalLength, kPrincipalPointY}, {0.0f, 0.0f, 1.0f}}};
}

std::vector<std::vector<float>> toVectorIntrinsics(const std::array<std::array<float, 3>, 3>& intrinsics) {
    return {
        {intrinsics[0][0], intrinsics[0][1], intrinsics[0][2]},
        {intrinsics[1][0], intrinsics[1][1], intrinsics[1][2]},
        {intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]},
    };
}

std::array<std::array<float, 3>, 3> makeScaledIntrinsics(float fx, float fy, float cx, float cy) {
    return {{{fx, 0.0f, cx}, {0.0f, fy, cy}, {0.0f, 0.0f, 1.0f}}};
}

std::shared_ptr<dai::ImgFrame> makeAlignToFrame() {
    auto intrinsics = makeIntrinsics();
    dai::Extrinsics extrinsics({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A);
    dai::ImgTransformation transformation(
        kDepthWidth, kDepthHeight, intrinsics, dai::CameraModel::Perspective, std::vector<float>(14, 0.0f), extrinsics);

    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(kDepthWidth);
    frame->setHeight(kDepthHeight);
    frame->setStride(kDepthWidth);
    frame->setType(dai::ImgFrame::Type::GRAY8);
    frame->setInstanceNum(static_cast<unsigned>(dai::CameraBoardSocket::CAM_A));
    frame->setTransformation(transformation);
    frame->setData(std::vector<uint8_t>(kDepthWidth * kDepthHeight, 127));
    return frame;
}

std::shared_ptr<dai::ImgFrame> makeAlignToFrameWithTransformationSize(
    unsigned transformWidth, unsigned transformHeight, const std::array<std::array<float, 3>, 3>& intrinsics) {
    dai::Extrinsics extrinsics({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A);
    dai::ImgTransformation transformation(kDepthWidth, kDepthHeight, intrinsics, dai::CameraModel::Perspective, std::vector<float>(14, 0.0f), extrinsics);
    transformation.setSize(transformWidth, transformHeight);

    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(kDepthWidth);
    frame->setHeight(kDepthHeight);
    frame->setStride(kDepthWidth);
    frame->setType(dai::ImgFrame::Type::GRAY8);
    frame->setInstanceNum(static_cast<unsigned>(dai::CameraBoardSocket::CAM_A));
    frame->setTransformation(transformation);
    frame->setData(std::vector<uint8_t>(kDepthWidth * kDepthHeight, 63));
    return frame;
}

void refreshAlignToTransformation(dai::ImgTransformation& transformation, unsigned alignWidth, unsigned alignHeight) {
    auto [transformWidth, transformHeight] = transformation.getSize();
    if(transformWidth != alignWidth || transformHeight != alignHeight) {
        const float scaleX = static_cast<float>(alignWidth) / static_cast<float>(transformWidth);
        const float scaleY = static_cast<float>(alignHeight) / static_cast<float>(transformHeight);
        transformation.addScale(scaleX, scaleY);
        transformation.setSize(alignWidth, alignHeight);
    }
}

std::shared_ptr<dai::ImgFrame> makeDepthFrame() {
    auto intrinsics = makeIntrinsics();
    dai::Extrinsics extrinsics({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_B);
    dai::ImgTransformation transformation(
        kDepthWidth, kDepthHeight, intrinsics, dai::CameraModel::Perspective, std::vector<float>(14, 0.0f), extrinsics);

    std::vector<uint16_t> depthValues(kDepthWidth * kDepthHeight, 0);
    for(unsigned y = 8; y < 40; ++y) {
        for(unsigned x = 14; x < 20; ++x) {
            depthValues[y * kDepthWidth + x] = 1000;
        }
        for(unsigned x = 34; x < 42; ++x) {
            depthValues[y * kDepthWidth + x] = 500;
        }
    }

    std::vector<uint8_t> depthBytes(depthValues.size() * sizeof(uint16_t));
    std::memcpy(depthBytes.data(), depthValues.data(), depthBytes.size());

    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(kDepthWidth);
    frame->setHeight(kDepthHeight);
    frame->setStride(kDepthWidth * sizeof(uint16_t));
    frame->setType(dai::ImgFrame::Type::RAW16);
    frame->setInstanceNum(static_cast<unsigned>(dai::CameraBoardSocket::CAM_B));
    frame->setTransformation(transformation);
    frame->setData(std::move(depthBytes));
    return frame;
}

dai::CalibrationHandler makeCalibration(float translationXcm) {
    dai::CalibrationHandler handler;
    auto eeprom = handler.getEepromData();
    eeprom.stereoUseSpecTranslation = false;
    eeprom.stereoEnableDistortionCorrection = true;
    handler = dai::CalibrationHandler(eeprom);

    auto intrinsics = toVectorIntrinsics(makeIntrinsics());
    auto distortion = std::vector<float>(14, 0.0f);
    auto identityRotation = std::vector<std::vector<float>>{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, intrinsics, kDepthWidth, kDepthHeight);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intrinsics, kDepthWidth, kDepthHeight);
    handler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_A, distortion);
    handler.setDistortionCoefficients(dai::CameraBoardSocket::CAM_B, distortion);
    handler.setCameraExtrinsics(
        dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_A, identityRotation, {translationXcm, 0.0f, 0.0f}, {translationXcm, 0.0f, 0.0f});
    return handler;
}

}  // namespace

TEST_CASE("ImageAlign runtime calibration update changes aligned depth output") {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);

    auto depthInQ = align->input.createInputQueue();
    auto alignToInQ = align->inputAlignTo.createInputQueue();
    auto alignedOutQ = align->outputAligned.createOutputQueue();

    auto initialCalibration = makeCalibration(2.0f);
    auto updatedCalibration = makeCalibration(5.0f);

    pipeline.getDefaultDevice()->setCalibration(initialCalibration);
    pipeline.start();

    alignToInQ->send(makeAlignToFrame());
    depthInQ->send(makeDepthFrame());

    auto firstAligned = alignedOutQ->get<dai::ImgFrame>();
    REQUIRE(firstAligned != nullptr);
    REQUIRE(firstAligned->getWidth() == kDepthWidth);
    REQUIRE(firstAligned->getHeight() == kDepthHeight);
    REQUIRE(firstAligned->getType() == dai::ImgFrame::Type::RAW16);
    REQUIRE(firstAligned->getInstanceNum() == static_cast<unsigned>(dai::CameraBoardSocket::CAM_A));

    pipeline.getDefaultDevice()->setCalibration(updatedCalibration);

    alignToInQ->send(makeAlignToFrame());
    depthInQ->send(makeDepthFrame());

    auto secondAligned = alignedOutQ->get<dai::ImgFrame>();
    REQUIRE(secondAligned != nullptr);
    REQUIRE(secondAligned->getWidth() == kDepthWidth);
    REQUIRE(secondAligned->getHeight() == kDepthHeight);
    REQUIRE(secondAligned->getType() == dai::ImgFrame::Type::RAW16);
    REQUIRE(secondAligned->getInstanceNum() == static_cast<unsigned>(dai::CameraBoardSocket::CAM_A));

    REQUIRE(firstAligned->transformation.isEqualTransformation(secondAligned->transformation));

    const auto firstData = firstAligned->getData();
    const auto secondData = secondAligned->getData();
    REQUIRE(firstData.size() == secondData.size());
    REQUIRE_FALSE(std::equal(firstData.begin(), firstData.end(), secondData.begin(), secondData.end()));

    pipeline.stop();
}

TEST_CASE("ImageAlign refreshes align-to intrinsics after runtime calibration update") {
    dai::Pipeline pipeline;
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);

    auto depthInQ = align->input.createInputQueue();
    auto alignToInQ = align->inputAlignTo.createInputQueue();
    auto alignedOutQ = align->outputAligned.createOutputQueue();

    const auto initialCalibration = makeCalibration(2.0f);
    const auto updatedCalibration = makeCalibration(5.0f);

    const auto preUpdateIntrinsics = makeScaledIntrinsics(80.0f, 84.0f, 15.0f, 11.0f);
    const auto postUpdateIntrinsics = makeScaledIntrinsics(124.0f, 118.0f, 19.0f, 17.0f);

    auto preUpdateAlignTo = makeAlignToFrameWithTransformationSize(32, 24, preUpdateIntrinsics);
    auto postUpdateAlignTo = makeAlignToFrameWithTransformationSize(48, 36, postUpdateIntrinsics);

    auto expectedPreTransform = preUpdateAlignTo->transformation;
    auto expectedPostTransform = postUpdateAlignTo->transformation;
    refreshAlignToTransformation(expectedPreTransform, kDepthWidth, kDepthHeight);
    refreshAlignToTransformation(expectedPostTransform, kDepthWidth, kDepthHeight);

    pipeline.getDefaultDevice()->setCalibration(initialCalibration);
    pipeline.start();

    alignToInQ->send(preUpdateAlignTo);
    depthInQ->send(makeDepthFrame());

    auto firstAligned = alignedOutQ->get<dai::ImgFrame>();
    REQUIRE(firstAligned != nullptr);
    REQUIRE(firstAligned->getWidth() == kDepthWidth);
    REQUIRE(firstAligned->getHeight() == kDepthHeight);
    REQUIRE(firstAligned->getInstanceNum() == static_cast<unsigned>(dai::CameraBoardSocket::CAM_A));
    REQUIRE(firstAligned->transformation.isEqualTransformation(expectedPreTransform));
    REQUIRE(firstAligned->transformation.getIntrinsicMatrix() == expectedPreTransform.getIntrinsicMatrix());

    pipeline.getDefaultDevice()->setCalibration(updatedCalibration);

    alignToInQ->send(postUpdateAlignTo);
    depthInQ->send(makeDepthFrame());

    auto secondAligned = alignedOutQ->get<dai::ImgFrame>();
    REQUIRE(secondAligned != nullptr);
    REQUIRE(secondAligned->getWidth() == kDepthWidth);
    REQUIRE(secondAligned->getHeight() == kDepthHeight);
    REQUIRE(secondAligned->getInstanceNum() == static_cast<unsigned>(dai::CameraBoardSocket::CAM_A));
    REQUIRE(secondAligned->transformation.isEqualTransformation(expectedPostTransform));
    REQUIRE(secondAligned->transformation.getIntrinsicMatrix() == expectedPostTransform.getIntrinsicMatrix());
    REQUIRE_FALSE(secondAligned->transformation.isEqualTransformation(expectedPreTransform));
    REQUIRE(secondAligned->transformation.getIntrinsicMatrix() != firstAligned->transformation.getIntrinsicMatrix());

    pipeline.stop();
}
