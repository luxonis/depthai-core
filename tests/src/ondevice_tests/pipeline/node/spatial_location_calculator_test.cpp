#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <tuple>
#include <vector>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

using Catch::Approx;

namespace {

void setDepthValue(cv::Mat& depth, int xStart, int yStart, int xEnd, int yEnd, std::uint16_t value) {
    for(int y = yStart; y < yEnd; ++y) {
        for(int x = xStart; x < xEnd; ++x) {
            depth.at<std::uint16_t>(y, x) = value;
        }
    }
}

std::shared_ptr<dai::ImgFrame> createDepthFrame(const cv::Mat& depthMat, const std::array<std::array<float, 3>, 3>& intrinsics) {
    const unsigned width = depthMat.cols;
    const unsigned height = depthMat.rows;
    auto depthFrame = std::make_shared<dai::ImgFrame>();
    depthFrame->setCvFrame(depthMat, dai::ImgFrame::Type::RAW16);
    depthFrame->setSourceSize(width, height);
    depthFrame->setTimestamp(std::chrono::steady_clock::now());
    depthFrame->setSequenceNum(0);
    depthFrame->transformation.setSourceSize(width, height);
    depthFrame->transformation.setSize(width, height);
    depthFrame->transformation.setIntrinsicMatrix(intrinsics);
    REQUIRE(depthFrame->validateTransformations());
    return depthFrame;
}

std::shared_ptr<dai::ImgFrame> createDetectionFrameWithManipulation(const std::shared_ptr<dai::ImgFrame>& depthFrame, unsigned width, unsigned height) {
    constexpr float rotationDegrees = 180.0F;
    constexpr std::array<float, 4> shearMatrix = {1.0F, 0.08F, 0.12F, 1.0F};

    dai::Pipeline pipeline;
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->setFrameType(depthFrame->getType());
    manip->initialConfig->setOutputSize(width, height);
    manip->initialConfig->addRotateDeg(rotationDegrees);
    manip->initialConfig->addTransformAffine(shearMatrix);

    auto inputQueue = manip->inputImage.createInputQueue();
    auto outputQueue = manip->out.createOutputQueue();

    pipeline.start();
    inputQueue->send(depthFrame);
    auto transformedFrame = outputQueue->get<dai::ImgFrame>();
    REQUIRE(transformedFrame != nullptr);
    pipeline.stop();
    pipeline.wait();

    return transformedFrame;
}

std::tuple<dai::SpatialLocationCalculatorData, dai::ImgFrame, dai::SpatialImgDetections> processDepthFrame(
    dai::SpatialLocationCalculatorConfig initialConfig,
    std::shared_ptr<dai::ImgFrame> depthMat,
    std::shared_ptr<dai::ImgDetections> detectionMsg = nullptr,
    std::optional<std::vector<dai::SpatialLocationCalculatorConfigData>> configData = std::nullopt) {
    dai::Pipeline pipeline;
    auto spatial = pipeline.create<dai::node::SpatialLocationCalculator>();

    spatial->initialConfig = std::make_shared<dai::SpatialLocationCalculatorConfig>(initialConfig);
    if(configData.has_value()) {
        spatial->initialConfig->setROIs(*configData);
    }

    auto depthQueue = spatial->inputDepth.createInputQueue();
    std::shared_ptr<dai::InputQueue> detectionsQueue = nullptr;
    if(detectionMsg != nullptr) {
        detectionsQueue = spatial->inputDetections.createInputQueue();
    }

    std::shared_ptr<dai::MessageQueue> outputQueue = nullptr;
    if(detectionMsg != nullptr) {
        outputQueue = spatial->outputDetections.createOutputQueue();
    }
    auto passthroughQueue = spatial->passthroughDepth.createOutputQueue();
    std::shared_ptr<dai::MessageQueue> legacyOutputQueue = nullptr;
    if(configData.has_value()) {
        legacyOutputQueue = spatial->out.createOutputQueue();
    }

    pipeline.start();
    std::shared_ptr<dai::SpatialLocationCalculatorData> legacySpatialData = nullptr;
    std::shared_ptr<dai::ImgFrame> passthroughFrame = nullptr;
    std::shared_ptr<dai::SpatialImgDetections> detectionOutput = nullptr;

    depthQueue->send(depthMat);
    if(detectionsQueue != nullptr) {
        detectionsQueue->send(detectionMsg);
    }
    std::shared_ptr<dai::SpatialImgDetections> spatialDetections = nullptr;

    passthroughFrame = passthroughQueue->get<dai::ImgFrame>();
    REQUIRE(passthroughFrame != nullptr);
    if(outputQueue != nullptr) {
        spatialDetections = outputQueue->get<dai::SpatialImgDetections>();
        REQUIRE(spatialDetections != nullptr);
    }

    if(legacyOutputQueue != nullptr) {
        legacySpatialData = legacyOutputQueue->get<dai::SpatialLocationCalculatorData>();
        REQUIRE(legacySpatialData != nullptr);
    }

    if(spatialDetections != nullptr) {
        detectionOutput = spatialDetections;
    }

    pipeline.stop();
    pipeline.wait();

    return std::make_tuple(configData.has_value() ? *legacySpatialData : dai::SpatialLocationCalculatorData(),
                           *passthroughFrame,
                           detectionOutput ? *detectionOutput : dai::SpatialImgDetections());
}

}  // namespace

TEST_CASE("SpatialLocationCalculatorConfig tracks ROI updates") {
    dai::SpatialLocationCalculatorConfig config;
    REQUIRE(config.getConfigData().empty());

    dai::SpatialLocationCalculatorConfigData initial;
    initial.roi = dai::Rect(10.0F, 20.0F, 30.0F, 40.0F, false);
    initial.depthThresholds.lowerThreshold = 100;
    initial.depthThresholds.upperThreshold = 2000;
    initial.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MODE;
    initial.stepSize = 1;
    config.addROI(initial);

    auto rois = config.getConfigData();
    REQUIRE(rois.size() == 1);
    CHECK(rois.front().depthThresholds.lowerThreshold == initial.depthThresholds.lowerThreshold);
    CHECK(rois.front().depthThresholds.upperThreshold == initial.depthThresholds.upperThreshold);
    CHECK(rois.front().calculationAlgorithm == initial.calculationAlgorithm);
    CHECK(rois.front().stepSize == initial.stepSize);
    CHECK(rois.front().roi.x == Approx(initial.roi.x));
    CHECK(rois.front().roi.y == Approx(initial.roi.y));
    CHECK(rois.front().roi.width == Approx(initial.roi.width));
    CHECK(rois.front().roi.height == Approx(initial.roi.height));

    dai::SpatialLocationCalculatorConfigData overrideA;
    overrideA.roi = dai::Rect(0.05F, 0.05F, 0.25F, 0.25F, true);
    overrideA.depthThresholds.lowerThreshold = 300;
    overrideA.depthThresholds.upperThreshold = 4500;
    overrideA.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MAX;
    overrideA.stepSize = 6;

    dai::SpatialLocationCalculatorConfigData overrideB;
    overrideB.roi = dai::Rect(0.3F, 0.4F, 0.2F, 0.15F, true);
    overrideB.depthThresholds.lowerThreshold = 200;
    overrideB.depthThresholds.upperThreshold = 5200;
    overrideB.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEDIAN;
    overrideB.stepSize = 3;

    config.setROIs({overrideA, overrideB});
    rois = config.getConfigData();
    REQUIRE(rois.size() == 2);

    CHECK(rois[0].depthThresholds.lowerThreshold == overrideA.depthThresholds.lowerThreshold);
    CHECK(rois[0].depthThresholds.upperThreshold == overrideA.depthThresholds.upperThreshold);
    CHECK(rois[0].calculationAlgorithm == overrideA.calculationAlgorithm);
    CHECK(rois[0].stepSize == overrideA.stepSize);
    CHECK(rois[0].roi.x == Approx(overrideA.roi.x));
    CHECK(rois[0].roi.y == Approx(overrideA.roi.y));
    CHECK(rois[0].roi.width == Approx(overrideA.roi.width));
    CHECK(rois[0].roi.height == Approx(overrideA.roi.height));

    CHECK(rois[1].depthThresholds.lowerThreshold == overrideB.depthThresholds.lowerThreshold);
    CHECK(rois[1].depthThresholds.upperThreshold == overrideB.depthThresholds.upperThreshold);
    CHECK(rois[1].calculationAlgorithm == overrideB.calculationAlgorithm);
    CHECK(rois[1].stepSize == overrideB.stepSize);
    CHECK(rois[1].roi.x == Approx(overrideB.roi.x));
    CHECK(rois[1].roi.y == Approx(overrideB.roi.y));
    CHECK(rois[1].roi.width == Approx(overrideB.roi.width));
    CHECK(rois[1].roi.height == Approx(overrideB.roi.height));
}

TEST_CASE("SpatialLocationCalculator synthetic depth data test") {
    constexpr unsigned width = 640;
    constexpr unsigned height = 480;

    struct RoiSpec {
        dai::Rect roi;
        std::uint16_t depth;
        std::uint32_t widthPx;
        std::uint32_t heightPx;
    };

    const std::vector<RoiSpec> roiSpecs = {
        {dai::Rect(100.0F, 120.0F, 40.0F, 60.0F, false), static_cast<std::uint16_t>(400), 40, 60},
        {dai::Rect(320.0F, 200.0F, 64.0F, 72.0F, false), static_cast<std::uint16_t>(600), 64, 72},
        {dai::Rect(420.0F, 80.0F, 96.0F, 96.0F, false), static_cast<std::uint16_t>(800), 96, 96},
    };

    std::vector<dai::SpatialLocationCalculatorConfigData> configData;
    configData.reserve(roiSpecs.size());
    for(const auto& spec : roiSpecs) {
        dai::SpatialLocationCalculatorConfigData cfg;
        cfg.roi = spec.roi;
        cfg.depthThresholds.lowerThreshold = 0;
        cfg.depthThresholds.upperThreshold = 10000;
        cfg.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::AVERAGE;
        cfg.stepSize = 1;
        configData.push_back(cfg);
    }

    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{7.5F, 0.0F, 3.2F}},
        {{0.0F, 4.8F, 2.4F}},
        {{0.0F, 0.0F, 1.0F}},
    }};
    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(1000));
    for(const auto& spec : roiSpecs) {
        const int x0 = static_cast<int>(spec.roi.x);
        const int y0 = static_cast<int>(spec.roi.y);
        setDepthValue(depthMat, x0, y0, x0 + static_cast<int>(spec.widthPx), y0 + static_cast<int>(spec.heightPx), spec.depth);
    }
    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig config;

    auto [spatialData, passthroughFrame, spatialDetections] = processDepthFrame(config, depthFrame, nullptr, configData);
    REQUIRE(spatialDetections.detections.empty());
    const auto results = spatialData.getSpatialLocations();
    REQUIRE(results.size() == roiSpecs.size());

    CHECK(passthroughFrame.getWidth() == width);
    CHECK(passthroughFrame.getHeight() == height);

    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];

    for(std::size_t idx = 0; idx < roiSpecs.size(); ++idx) {
        const auto& spec = roiSpecs[idx];
        const auto& spatialLoc = results.at(idx);

        CHECK(spatialLoc.depthAverage == Approx(static_cast<float>(spec.depth)).margin(1.0F));
        CHECK(spatialLoc.depthMin == spec.depth);
        CHECK(spatialLoc.depthMax == spec.depth);
        CHECK(spatialLoc.depthAveragePixelCount == spec.widthPx * spec.heightPx);

        const float centerX = spec.roi.x + spec.roi.width / 2.0F;
        const float centerY = spec.roi.y + spec.roi.height / 2.0F;
        const float expectedX = ((centerX - cx) / fx) * static_cast<float>(spec.depth);
        const float expectedY = ((centerY - cy) / fy) * static_cast<float>(spec.depth);
        const float expectedZ = static_cast<float>(spec.depth);

        CHECK(spatialLoc.spatialCoordinates.x == Approx(expectedX).margin(5.0F));
        CHECK(spatialLoc.spatialCoordinates.y == Approx(expectedY).margin(5.0F));
        CHECK(spatialLoc.spatialCoordinates.z == Approx(expectedZ).margin(1.0F));
    }
}

TEST_CASE("Spatial keypoints support") {
    constexpr unsigned width = 800;
    constexpr unsigned height = 600;

    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{7.5F, 0.0F, 3.2F}},
        {{0.0F, 4.8F, 2.4F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    constexpr int kpRadius = 10;
    const int kp0x = 120;
    const int kp0y = 200;
    const int kp1x = 500;
    const int kp1y = 300;

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(3000));
    setDepthValue(depthMat, kp0x - kpRadius, kp0y - kpRadius, kp0x + kpRadius, kp0y + kpRadius, 1800);
    setDepthValue(depthMat, kp1x - kpRadius, kp1y - kpRadius, kp1x + kpRadius, kp1y + kpRadius, 11000);
    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    constexpr std::uint16_t maxDepthThreshold = 10000;
    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(0, maxDepthThreshold);
    initialConfig.setKeypointRadius(kpRadius);
    initialConfig.setCalculateSpatialKeypoints(true);
    initialConfig.setUseSegmentation(false);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f(0.5F, 0.5F, true), dai::Size2f(0.8F, 0.8F, true), 0.0F));
    detection.confidence = 0.95F;
    detection.label = 11;
    std::vector<dai::Keypoint> keypoints;
    keypoints.emplace_back(dai::Point2f(static_cast<float>(kp0x) / width, static_cast<float>(kp0y) / height, true));
    keypoints.emplace_back(dai::Point2f(static_cast<float>(kp1x) / width, static_cast<float>(kp1y) / height, true));
    detection.setKeypoints(keypoints);

    detectionMsg->transformation = depthFrame->transformation;
    detectionMsg->setTimestamp(depthFrame->getTimestamp());
    detectionMsg->setSequenceNum(depthFrame->getSequenceNum());
    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);
    REQUIRE(spatialDetections.detections.size() == 1);

    const auto& spatialDetection = spatialDetections.detections.front();
    const auto spatialKeypoints = spatialDetection.getKeypoints();
    REQUIRE(spatialKeypoints.size() == keypoints.size());

    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];

    auto computeExpectedDepth = [&](int pxCenter, int pyCenter) {
        const int startX = std::max(0, pxCenter - kpRadius);
        const int startY = std::max(0, pyCenter - kpRadius);
        const int endX = std::min(static_cast<int>(width) - 1, pxCenter + kpRadius);
        const int endY = std::min(static_cast<int>(height) - 1, pyCenter + kpRadius);

        std::uint32_t sum = 0;
        std::uint32_t count = 0;
        for(int y = startY; y < endY; ++y) {
            for(int x = startX; x < endX; ++x) {
                const float dist = std::hypot(static_cast<float>(pxCenter - x), static_cast<float>(pyCenter - y));
                if(dist <= kpRadius) {
                    const std::uint16_t px = depthMat.at<std::uint16_t>(y, x);
                    if(px > 0 && px < maxDepthThreshold) {
                        sum += px;
                        ++count;
                    }
                }
            }
        }
        REQUIRE(count > 0);
        return static_cast<float>(sum) / static_cast<float>(count);
    };

    const float kp0DepthAvg = computeExpectedDepth(kp0x, kp0y);
    auto expectCoord = [&](int px, int py, float depthMm) {
        const float expectedX = (static_cast<float>(px) - cx) * depthMm / fx;
        const float expectedY = (static_cast<float>(py) - cy) * depthMm / fy;
        return std::array<float, 3>{expectedX, expectedY, depthMm};
    };

    const auto kp0Expected = expectCoord(kp0x, kp0y, kp0DepthAvg);
    CHECK(spatialKeypoints[0].spatialCoordinates.x == Approx(kp0Expected[0]).margin(1.0F));
    CHECK(spatialKeypoints[0].spatialCoordinates.y == Approx(kp0Expected[1]).margin(1.0F));
    CHECK(spatialKeypoints[0].spatialCoordinates.z == Approx(kp0Expected[2]).margin(1.0F));

    CHECK(spatialKeypoints[1].spatialCoordinates.z == 0);
}

TEST_CASE("Spatial detections respect segmentation mask pixels") {
    constexpr unsigned width = 800;
    constexpr unsigned height = 600;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{7.5F, 0.0F, 3.2F}},
        {{0.0F, 4.8F, 2.4F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(3100));
    const int bboxStartX = 150;
    const int bboxStartY = 120;
    const int bboxWidth = 420;
    const int bboxHeight = 300;
    setDepthValue(depthMat, bboxStartX, bboxStartY, bboxStartX + bboxWidth, bboxStartY + bboxHeight, static_cast<std::uint16_t>(2600));

    const int segStartX = bboxStartX + 80;
    const int segStartY = bboxStartY + 60;
    const int segWidth = 160;
    const int segHeight = 140;
    std::vector<std::pair<int, int>> segmentationPixels;
    segmentationPixels.reserve(segWidth * segHeight);
    for(int y = segStartY; y < segStartY + segHeight; ++y) {
        for(int x = segStartX; x < segStartX + segWidth; ++x) {
            segmentationPixels.emplace_back(x, y);
            depthMat.at<std::uint16_t>(y, x) = static_cast<std::uint16_t>(1400);
        }
    }

    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(0, 10000);
    initialConfig.setUseSegmentation(true);
    initialConfig.setCalculateSpatialKeypoints(false);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    const float normWidth = static_cast<float>(bboxWidth) / static_cast<float>(width);
    const float normHeight = static_cast<float>(bboxHeight) / static_cast<float>(height);
    const float normCenterX = static_cast<float>(bboxStartX) / width + normWidth / 2.0F;
    const float normCenterY = static_cast<float>(bboxStartY) / height + normHeight / 2.0F;
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f(normCenterX, normCenterY, true), dai::Size2f(normWidth, normHeight, true), 0.0F));
    detection.confidence = 0.7F;
    detection.label = 0;

    std::vector<std::uint8_t> mask(width * height, 255);
    for(const auto& [x, y] : segmentationPixels) {
        mask.at(y * static_cast<int>(width) + x) = 0;
    }
    detectionMsg->setSegmentationMask(mask, width, height);
    detectionMsg->transformation = depthFrame->transformation;
    detectionMsg->setTimestamp(depthFrame->getTimestamp());
    detectionMsg->setSequenceNum(depthFrame->getSequenceNum());

    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);

    REQUIRE(spatialDetections.detections.size() == 1);

    const auto& spatialDetection = spatialDetections.detections.front();
    const float expectedDepth = 1400.0F;
    CHECK(spatialDetection.spatialCoordinates.z == Approx(expectedDepth).margin(1.0F));

    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];
    const float centerXpx = static_cast<float>(bboxStartX) + static_cast<float>(bboxWidth) / 2.0F;
    const float centerYpx = static_cast<float>(bboxStartY) + static_cast<float>(bboxHeight) / 2.0F;
    const float expectedX = (centerXpx - cx) * expectedDepth / fx;
    const float expectedY = (centerYpx - cy) * expectedDepth / fy;
    CHECK(spatialDetection.spatialCoordinates.x == Approx(expectedX).margin(1.0F));
    CHECK(spatialDetection.spatialCoordinates.y == Approx(expectedY).margin(1.0F));

    REQUIRE(spatialDetection.boundingBoxMapping.depthThresholds.lowerThreshold == initialConfig.globalLowerThreshold);
    REQUIRE(spatialDetection.boundingBoxMapping.depthThresholds.upperThreshold == initialConfig.globalUpperThreshold);
    REQUIRE(spatialDetection.boundingBoxMapping.calculationAlgorithm == initialConfig.globalCalculationAlgorithm);
}

TEST_CASE("Segmentation usage can be toggled") {
    constexpr unsigned width = 640;
    constexpr unsigned height = 480;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{7.5F, 0.0F, 3.2F}},
        {{0.0F, 4.8F, 2.4F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(3000));
    const int bboxStartX = 100;
    const int bboxStartY = 80;
    const int bboxWidth = 120;
    const int bboxHeight = 80;
    setDepthValue(depthMat, bboxStartX, bboxStartY, bboxStartX + bboxWidth, bboxStartY + bboxHeight, 2600);

    const int segStartX = bboxStartX + 20;
    const int segStartY = bboxStartY + 20;
    const int segWidth = 40;
    const int segHeight = 30;
    setDepthValue(depthMat, segStartX, segStartY, segStartX + segWidth, segStartY + segHeight, 1400);

    auto runSpatial = [&](bool useSegmentation) {
        auto depthFrame = createDepthFrame(depthMat, intrinsics);
        dai::SpatialLocationCalculatorConfig initialConfig;
        initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
        initialConfig.setDepthThresholds(0, 10000);
        initialConfig.setUseSegmentation(useSegmentation);
        initialConfig.setCalculateSpatialKeypoints(false);

        auto detectionMsg = std::make_shared<dai::ImgDetections>();
        detectionMsg->detections.resize(1);
        dai::ImgDetection detection;
        const float normWidth = static_cast<float>(bboxWidth) / static_cast<float>(width);
        const float normHeight = static_cast<float>(bboxHeight) / static_cast<float>(height);
        const float normCenterX = static_cast<float>(bboxStartX) / width + normWidth / 2.0F;
        const float normCenterY = static_cast<float>(bboxStartY) / height + normHeight / 2.0F;
        detection.setBoundingBox(dai::RotatedRect(dai::Point2f(normCenterX, normCenterY, true), dai::Size2f(normWidth, normHeight, true), 0.0F));
        detection.confidence = 0.6F;
        detection.label = 1;
        detectionMsg->detections[0] = detection;

        std::vector<std::uint8_t> mask(width * height, 255);
        for(int y = segStartY; y < segStartY + segHeight; ++y) {
            for(int x = segStartX; x < segStartX + segWidth; ++x) {
                mask.at(y * static_cast<int>(width) + x) = 0;
            }
        }
        detectionMsg->setSegmentationMask(mask, width, height);
        detectionMsg->transformation = depthFrame->transformation;
        detectionMsg->setTimestamp(depthFrame->getTimestamp());
        detectionMsg->setSequenceNum(depthFrame->getSequenceNum());

        auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
        static_cast<void>(unusedLegacy);
        static_cast<void>(unusedPassthrough);
        REQUIRE(spatialDetections.detections.size() == 1);
        return spatialDetections.detections.front();
    };

    SECTION("Segmentation enabled uses mask pixels") {
        const auto detection = runSpatial(true);
        CHECK(detection.spatialCoordinates.z == Approx(1400.0F).margin(1.0F));
    }
    SECTION("Segmentation disabled falls back to bounding box averaging") {
        const auto detection = runSpatial(false);
        const float segArea = static_cast<float>(segWidth * segHeight);
        const float bboxArea = static_cast<float>(bboxWidth * bboxHeight);
        const float expectedDepth = ((segArea * 1400.0F) + ((bboxArea - segArea) * 2600.0F)) / bboxArea;  // mask ignored, so full bbox average applies
        CHECK(detection.spatialCoordinates.z == Approx(expectedDepth).margin(1.0F));
    }
}

TEST_CASE("Spatial detections remap depth to detection transformations") {
    constexpr unsigned width = 320;
    constexpr unsigned height = 240;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{5.5F, 0.0F, 2.5F}},
        {{0.0F, 5.5F, 2.0F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    dai::Device device = dai::Device();
    if(device.getPlatform() == dai::Platform::RVC2) {  // skipping test on RVC2
        return;
    }

    constexpr std::uint16_t farDepth = 4200;
    constexpr std::uint16_t nearDepth = 1400;
    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(farDepth));
    setDepthValue(depthMat, 0, 0, static_cast<int>(width / 2), static_cast<int>(height), nearDepth);
    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(0, 10000);
    initialConfig.setUseSegmentation(false);
    initialConfig.setCalculateSpatialKeypoints(false);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    const float bboxWidthNorm = 0.2F;
    const float bboxHeightNorm = 0.5F;
    const float bboxCenterXNorm = 0.75F;
    const float bboxCenterYNorm = 0.5F;
    const dai::RotatedRect gtBbox{dai::Point2f(bboxCenterXNorm, bboxCenterYNorm, true), dai::Size2f(bboxWidthNorm, bboxHeightNorm, true), 0.0F};
    const dai::RotatedRect denormGtBbox = gtBbox.denormalize(width, height);
    detection.setBoundingBox(gtBbox);
    detection.confidence = 0.9F;
    detection.label = 7;
    auto manipulatedDetectionFrame = createDetectionFrameWithManipulation(depthFrame, width, height);
    detectionMsg->transformation = manipulatedDetectionFrame->transformation;
    detectionMsg->setTimestamp(manipulatedDetectionFrame->getTimestamp());
    detectionMsg->setSequenceNum(manipulatedDetectionFrame->getSequenceNum());

    const auto detectionBoundingBox = detection.getBoundingBox();
    REQUIRE(detectionMsg->transformation.has_value());
    const auto& detectionTransformation = detectionMsg->transformation.value();
    auto remappedRect = detectionTransformation.remapRectTo(depthFrame->transformation, detectionBoundingBox);
    auto remappedRectPx = remappedRect.denormalize(width, height);
    const float remappedCenterX = remappedRectPx.center.x;
    const float remappedCenterY = remappedRectPx.center.y;
    REQUIRE(remappedCenterX < static_cast<float>(width) / 2.0F);  // should land on the near-depth side

    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);
    REQUIRE(spatialDetections.detections.size() == 1);

    const auto formatMatrix = [](const std::array<std::array<float, 3>, 3>& matrix) {
        std::ostringstream oss;
        oss << '[';
        for(size_t row = 0; row < matrix.size(); ++row) {
            if(row > 0) {
                oss << ", ";
            }
            oss << '[' << matrix[row][0] << ", " << matrix[row][1] << ", " << matrix[row][2] << ']';
        }
        oss << ']';
        return oss.str();
    };
    INFO("Depth transformation matrix: " << formatMatrix(depthFrame->transformation.getMatrix()));
    INFO("RGB transformation matrix: " << formatMatrix(detectionTransformation.getMatrix()));

    const auto& spatialDetection = spatialDetections.detections.front();
    const auto intrinsicMatrix = depthFrame->transformation.getIntrinsicMatrix();
    const float fx = intrinsicMatrix[0][0];
    const float fy = intrinsicMatrix[1][1];
    const float cx = intrinsicMatrix[0][2];
    const float cy = intrinsicMatrix[1][2];
    const dai::RotatedRect denormalizedBBox = detection.getBoundingBox().denormalize(width, height);

    const float expectedDepth = static_cast<float>(nearDepth);
    const float expectedX = (denormGtBbox.center.x - cx) * expectedDepth / fx;
    const float expectedY = (denormGtBbox.center.y - cy) * expectedDepth / fy;

    // Check bounding box mapping
    const auto outputBox = spatialDetection.getBoundingBox();
    CHECK(outputBox.center.x == Approx(bboxCenterXNorm).margin(1e-6F));
    CHECK(outputBox.center.y == Approx(bboxCenterYNorm).margin(1e-6F));

    // Check spatial coordinates
    CHECK(spatialDetection.spatialCoordinates.z == Approx(expectedDepth).margin(1.0F));
    CHECK(spatialDetection.spatialCoordinates.x == Approx(expectedX).margin(1.0F));
    CHECK(spatialDetection.spatialCoordinates.y == Approx(expectedY).margin(1.0F));
}

TEST_CASE("Spatial detections return zero depth when depth frame is empty") {
    constexpr unsigned width = 160;
    constexpr unsigned height = 120;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{4.0F, 0.0F, 1.0F}},
        {{0.0F, 4.0F, 1.0F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(0));
    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(0, 5000);
    initialConfig.setUseSegmentation(false);
    initialConfig.setCalculateSpatialKeypoints(false);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f(0.5F, 0.5F, true), dai::Size2f(0.6F, 0.6F, true), 0.0F));
    detection.confidence = 0.5F;
    detection.label = 5;
    detectionMsg->transformation = depthFrame->transformation;
    detectionMsg->setTimestamp(depthFrame->getTimestamp());
    detectionMsg->setSequenceNum(depthFrame->getSequenceNum());

    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);
    REQUIRE(spatialDetections.detections.size() == 1);

    const auto& spatialDetection = spatialDetections.detections.front();
    CHECK(spatialDetection.spatialCoordinates.z == Approx(0.0F).margin(1.0F));
}

TEST_CASE("Spatial keypoints can be disabled") {
    constexpr unsigned width = 320;
    constexpr unsigned height = 240;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{5.0F, 0.0F, 1.0F}},
        {{0.0F, 5.0F, 1.0F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(2200));
    const int kpX = 100;
    const int kpY = 80;
    setDepthValue(depthMat, kpX - 5, kpY - 5, kpX + 5, kpY + 5, 1200);

    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(0, 10000);
    initialConfig.setKeypointRadius(6);
    initialConfig.setCalculateSpatialKeypoints(false);
    initialConfig.setUseSegmentation(false);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f(0.5F, 0.5F, true), dai::Size2f(0.5F, 0.5F, true), 0.0F));
    detection.confidence = 0.9F;
    detection.label = 2;
    detection.setKeypoints({dai::Keypoint(dai::Point2f(static_cast<float>(kpX) / width, static_cast<float>(kpY) / height, true))});

    detectionMsg->transformation = depthFrame->transformation;
    detectionMsg->setTimestamp(depthFrame->getTimestamp());
    detectionMsg->setSequenceNum(depthFrame->getSequenceNum());

    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);
    REQUIRE(spatialDetections.detections.size() == 1);

    const auto& spatialDetection = spatialDetections.detections.front();
    CHECK(spatialDetection.getKeypoints().empty());

    const int bboxStartX = static_cast<int>(width * 0.25F);
    const int bboxStartY = static_cast<int>(height * 0.25F);
    const int bboxWidth = static_cast<int>(width * 0.5F);
    const int bboxHeight = static_cast<int>(height * 0.5F);
    auto computeBboxAverage = [&]() {
        std::uint64_t sum = 0;
        std::uint64_t count = 0;
        for(int y = bboxStartY; y < bboxStartY + bboxHeight; ++y) {
            for(int x = bboxStartX; x < bboxStartX + bboxWidth; ++x) {
                sum += depthMat.at<std::uint16_t>(y, x);
                ++count;
            }
        }
        return static_cast<float>(sum) / static_cast<float>(count);
    };
    const float expectedZ = computeBboxAverage();
    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];
    const float centerXpx = static_cast<float>(bboxStartX + bboxWidth / 2.0F);
    const float centerYpx = static_cast<float>(bboxStartY + bboxHeight / 2.0F);
    const float expectedX = (centerXpx - cx) * expectedZ / fx;
    const float expectedY = (centerYpx - cy) * expectedZ / fy;
    REQUIRE(spatialDetection.spatialCoordinates.x == Approx(expectedX).margin(1.0F));
    REQUIRE(spatialDetection.spatialCoordinates.y == Approx(expectedY).margin(1.0F));
    REQUIRE(spatialDetection.spatialCoordinates.z == Approx(expectedZ).margin(1.0F));
}

TEST_CASE("Keypoint spatial calculation skips invalid or thresholded depth") {
    constexpr unsigned width = 200;
    constexpr unsigned height = 150;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{6.0F, 0.0F, 2.0F}},
        {{0.0F, 6.0F, 2.0F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(0));
    const int kpX = 80;
    const int kpY = 60;
    setDepthValue(depthMat, kpX - 8, kpY - 8, kpX, kpY, 4500);  // outside upper threshold
    setDepthValue(depthMat, kpX, kpY, kpX + 8, kpY + 8, 0);     // outside lower threshold
    setDepthValue(depthMat, kpX - 3, kpY - 3, kpX + 3, kpY + 3, 1500);

    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(100, 2000);
    initialConfig.setKeypointRadius(8);
    initialConfig.setCalculateSpatialKeypoints(true);
    initialConfig.setUseSegmentation(false);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f(0.4F, 0.4F, true), dai::Size2f(0.4F, 0.4F, true), 0.0F));
    detection.confidence = 0.8F;
    detection.label = 3;
    detection.setKeypoints({dai::Keypoint(dai::Point2f(static_cast<float>(kpX) / width, static_cast<float>(kpY) / height, true))});
    detectionMsg->transformation = depthFrame->transformation;
    detectionMsg->setTimestamp(depthFrame->getTimestamp());
    detectionMsg->setSequenceNum(depthFrame->getSequenceNum());

    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);
    REQUIRE(spatialDetections.detections.size() == 1);

    const auto& det = spatialDetections.detections.front();
    const auto kp = det.getKeypoints().at(0);
    REQUIRE(kp.spatialCoordinates.z == Approx(1500.0F).margin(1.0F));
}

TEST_CASE("Spatial detections handle segmentation and keypoints together") {
    constexpr unsigned width = 800;
    constexpr unsigned height = 600;
    const std::array<std::array<float, 3>, 3> intrinsics = {{
        {{7.0F, 0.0F, 3.0F}},
        {{0.0F, 7.0F, 2.0F}},
        {{0.0F, 0.0F, 1.0F}},
    }};

    cv::Mat depthMat(height, width, CV_16UC1, cv::Scalar(3200));
    const int bboxStartX = 60;
    const int bboxStartY = 40;
    const int bboxWidth = 200;
    const int bboxHeight = 160;
    setDepthValue(depthMat, bboxStartX, bboxStartY, bboxStartX + bboxWidth, bboxStartY + bboxHeight, 2800);

    // Segmentation mask region with closer depth
    const int segStartX = bboxStartX + 40;
    const int segStartY = bboxStartY + 50;
    const int segWidth = 60;
    const int segHeight = 50;
    setDepthValue(depthMat, segStartX, segStartY, segStartX + segWidth, segStartY + segHeight, 1200);

    // Keypoint located outside the segmentation blob, with different depth
    const int kpX = bboxStartX + 20;
    const int kpY = bboxStartY + 20;
    setDepthValue(depthMat, kpX - 5, kpY - 5, kpX + 5, kpY + 5, 2200);

    auto depthFrame = createDepthFrame(depthMat, intrinsics);

    dai::SpatialLocationCalculatorConfig initialConfig;
    initialConfig.setCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::AVERAGE);
    initialConfig.setDepthThresholds(0, 10000);
    initialConfig.setKeypointRadius(6);
    initialConfig.setCalculateSpatialKeypoints(true);
    initialConfig.setUseSegmentation(true);

    auto detectionMsg = std::make_shared<dai::ImgDetections>();
    detectionMsg->detections.resize(1);
    auto& detection = detectionMsg->detections.front();
    const float normWidth = static_cast<float>(bboxWidth) / static_cast<float>(width);
    const float normHeight = static_cast<float>(bboxHeight) / static_cast<float>(height);
    const float normCenterX = static_cast<float>(bboxStartX) / width + normWidth / 2.0F;
    const float normCenterY = static_cast<float>(bboxStartY) / height + normHeight / 2.0F;
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f(normCenterX, normCenterY, true), dai::Size2f(normWidth, normHeight, true), 0.0F));
    detection.confidence = 0.85F;
    detection.label = 4;

    detection.setKeypoints({dai::Keypoint(dai::Point2f(static_cast<float>(kpX) / width, static_cast<float>(kpY) / height, true))});

    std::vector<std::uint8_t> mask(width * height, 255);
    for(int y = segStartY; y < segStartY + segHeight; ++y) {
        for(int x = segStartX; x < segStartX + segWidth; ++x) {
            mask.at(y * static_cast<int>(width) + x) = 0;
        }
    }
    detectionMsg->setSegmentationMask(mask, width, height);
    detectionMsg->transformation = depthFrame->transformation;
    detectionMsg->setTimestamp(depthFrame->getTimestamp());
    detectionMsg->setSequenceNum(depthFrame->getSequenceNum());

    auto [unusedLegacy, unusedPassthrough, spatialDetections] = processDepthFrame(initialConfig, depthFrame, detectionMsg);
    static_cast<void>(unusedLegacy);
    static_cast<void>(unusedPassthrough);
    REQUIRE(spatialDetections.detections.size() == 1);

    const auto& spatialDetection = spatialDetections.detections.front();
    CHECK(spatialDetection.spatialCoordinates.z == Approx(1200.0F).margin(1.0F));

    const auto keypoints = spatialDetection.getKeypoints();
    const auto& kp = keypoints.at(0);
    REQUIRE(kp.spatialCoordinates.z == Approx(0).margin(1.0F));
}
