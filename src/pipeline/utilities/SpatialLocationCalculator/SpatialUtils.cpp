#include "SpatialUtils.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <optional>
#include <stdexcept>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/SpatialKeypoint.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace utilities {
namespace SpatialUtils {

float calculateDepth(
    SpatialLocationCalculatorAlgorithm algo, std::vector<uint16_t>& validPixels, std::uint32_t sum, std::uint32_t counter, float min, float max) {
    if(counter == 0 || validPixels.empty()) {
        return 0;
    }

    float z = 0;
    size_t midIndex = validPixels.size() / 2;
    switch(algo) {
        default:
        case SpatialLocationCalculatorAlgorithm::MEDIAN:
            std::nth_element(validPixels.begin(), validPixels.begin() + midIndex, validPixels.end());
            z = validPixels[midIndex];
            break;
        case SpatialLocationCalculatorAlgorithm::AVERAGE:
            z = (float)sum / counter;
            break;
        case SpatialLocationCalculatorAlgorithm::MIN:
            z = min;
            break;
        case SpatialLocationCalculatorAlgorithm::MAX:
            z = max;
            break;
        case SpatialLocationCalculatorAlgorithm::MODE: {
            constexpr std::size_t MAX_HISTOGRAM_RANGE = 4096;
            uint16_t minVal = static_cast<uint16_t>(min);
            uint16_t maxVal = static_cast<uint16_t>(max);
            if(maxVal < minVal) {
                z = 0;
                break;
            }
            std::size_t valueRange = static_cast<std::size_t>(maxVal - minVal) + 1;

            if(valueRange <= MAX_HISTOGRAM_RANGE) {
                // Small range: build a dense histogram for cache-friendly mode lookup
                std::vector<uint32_t> histogram(valueRange, 0);
                for(const auto& px : validPixels) {
                    if(px < minVal || px > maxVal) {
                        continue;
                    }
                    histogram[px - minVal]++;
                }

                uint32_t maxCount = 0;
                uint16_t modeValue = minVal;
                for(std::size_t i = 0; i < histogram.size(); ++i) {
                    if(histogram[i] > maxCount) {
                        maxCount = histogram[i];
                        modeValue = static_cast<uint16_t>(minVal + i);
                    }
                }
                z = maxCount ? modeValue : 0;
            } else {
                std::sort(validPixels.begin(), validPixels.end());

                uint16_t modeValue = validPixels[0];
                uint32_t maxCount = 1;
                uint32_t currentCount = 1;

                for(std::size_t i = 1; i < validPixels.size(); ++i) {
                    if(validPixels[i] == validPixels[i - 1]) {
                        currentCount++;
                    } else {
                        if(currentCount > maxCount) {
                            maxCount = currentCount;
                            modeValue = validPixels[i - 1];
                        }
                        currentCount = 1;
                    }
                }

                if(currentCount > maxCount) {
                    modeValue = validPixels.back();
                }

                z = modeValue;
            }
            break;
        }
    }
    return z;
}

dai::Point3f calculateSpatialCoordinates(float z, std::array<std::array<float, 3>, 3> intrinsicMatrix, dai::Point2f center) {
    float fx = intrinsicMatrix[0][0];
    float fy = intrinsicMatrix[1][1];
    float cx = intrinsicMatrix[0][2];
    float cy = intrinsicMatrix[1][2];

    float x = (center.x - cx) * z / fx;
    float y = (center.y - cy) * z / fy;

    return dai::Point3f(x, y, z);
}

void computeSpatialData(std::shared_ptr<dai::ImgFrame> depthFrame,
                        const std::vector<dai::SpatialLocationCalculatorConfigData>& configDataVec,
                        std::vector<dai::SpatialLocations>& spatialLocations,
                        std::shared_ptr<spdlog::async_logger> logger) {
    logger->info("Compute spatial data!");
    if(configDataVec.size() == 0) {
        logger->info("Empty config vector!");
        return;
    }

    if(!depthFrame->transformation.isValid()) {
        logger->error("DepthFrame has invalid Image Transformations. Cannot get intrinsic matrix, skipping computation of spatial coordinates.");
        return;
    }

    spatialLocations.resize(configDataVec.size());
    std::int32_t depthWidth = depthFrame->getWidth();
    std::int32_t depthHeight = depthFrame->getHeight();
    for(int i = 0; i < static_cast<int>(configDataVec.size()); i++) {
        const auto& cfg = configDataVec[i];
        const auto& roi = cfg.roi;
        float maxWidthSize = 1;
        float maxHeightSize = 1;
        const SpatialLocationCalculatorAlgorithm calcAlgo = cfg.calculationAlgorithm;

        int stepSize = cfg.stepSize;
        if(stepSize == SpatialLocationCalculatorConfigData::AUTO) {
            if(calcAlgo == SpatialLocationCalculatorAlgorithm::MODE || calcAlgo == SpatialLocationCalculatorAlgorithm::MEDIAN) {
                stepSize = 2;
            } else {
                stepSize = 1;
            }
        }

        if(!roi.isNormalized()) {
            maxWidthSize = depthFrame->getWidth();
            maxHeightSize = depthFrame->getHeight();
        }
        if((roi.x + roi.width > maxWidthSize) || (roi.y + roi.height > maxHeightSize)) {
            logger->error("ROI x:{} y:{} width:{} height:{} is not a valid rectangle. Out of depth map range: {}x{}",
                          roi.x,
                          roi.y,
                          roi.width,
                          roi.height,
                          maxWidthSize,
                          maxHeightSize);
            spatialLocations[i] = SpatialLocations();
            continue;
        }
        std::uint32_t lowerThreshold = cfg.depthThresholds.lowerThreshold;
        std::uint32_t upperThreshold = cfg.depthThresholds.upperThreshold;
        logger->debug("Depth limits, lower: {}, upper: {}", lowerThreshold, upperThreshold);

        bool skipCalculation = false;
        std::uint32_t sum = 0;
        std::uint16_t min = 65535, max = 0;
        std::uint32_t counter = 0;
        dai::Rect denormRoi = roi.denormalize(depthWidth, depthHeight);
        int ystart = denormRoi.topLeft().y;
        int yend = denormRoi.bottomRight().y;
        int xstart = denormRoi.topLeft().x;
        int xend = denormRoi.bottomRight().x;

        if(ystart > depthHeight || yend > depthHeight || xstart > depthWidth || xend > depthWidth) {
            skipCalculation = true;
            logger->warn("Invalid ROI. Depth image size: {}, {}. Bounding box: start - {},{} | end - {},{}. Skipping calculation.",
                         depthWidth,
                         depthHeight,
                         xstart,
                         ystart,
                         xend,
                         yend);
        }
        if(ystart >= yend || xstart >= xend) {
            skipCalculation = true;
            logger->warn("Invalid ROI. Bounding box: start - {},{} | end - {},{}. Skipping calculation.", xstart, ystart, xend, yend);
        }

        std::vector<uint16_t> validPixels;
        if(!skipCalculation) {
            uint16_t* plane = (uint16_t*)depthFrame->data->getData().data();
            for(int y = ystart; y < yend; y += stepSize) {
                for(int x = xstart; x < xend; x += stepSize) {
                    uint16_t px = plane[y * depthWidth + x];
                    if(px > lowerThreshold && px < upperThreshold) {
                        validPixels.push_back(px);
                        sum += px;
                        ++counter;
                        // min, max
                        if(px < min) min = px;
                        if(px > max) max = px;
                    }
                }
            }
        }
        float average = (counter == 0) ? 0 : (float)sum / counter;
        if(counter == 0) {
            min = max = 0;
        }

        SpatialLocations spatialData = {};
        spatialData.depthAverage = average;
        spatialData.depthMin = min;
        spatialData.depthMax = max;

        spatialData.config = cfg;
        spatialData.depthAveragePixelCount = counter;

        if(!skipCalculation) {
            float z = calculateDepth(calcAlgo, validPixels, sum, counter, min, max);

            float roiCx = denormRoi.x + denormRoi.width / 2.0f;
            float roiCy = denormRoi.y + denormRoi.height / 2.0f;
            std::array<std::array<float, 3>, 3> intrinsicMatrix = depthFrame->transformation.getIntrinsicMatrix();
            dai::Point3f spatialCoordinates = calculateSpatialCoordinates(z, intrinsicMatrix, dai::Point2f(roiCx, roiCy));
            logger->debug("Calculated spatial coordinates: {} {} {}", spatialCoordinates.x, spatialCoordinates.y, spatialCoordinates.z);

            spatialData.spatialCoordinates = spatialCoordinates;
        }

        spatialLocations[i] = spatialData;
    }
}

void computeSpatialDetections(std::shared_ptr<dai::ImgFrame> depthFrame,
                              const std::shared_ptr<SpatialLocationCalculatorConfig> config,
                              dai::ImgDetections& imgDetections,
                              dai::SpatialImgDetections& spatialDetections,
                              std::shared_ptr<spdlog::async_logger> logger) {
    logger->info("Computing spatial image detections!");

    if(!imgDetections.transformation.has_value()) {
        throw std::runtime_error("No transformation set on ImgDetections. Cannot compute spatial coordinates.");
    }

    std::vector<dai::ImgDetection> imgDetectionsVector = imgDetections.detections;
    if(imgDetectionsVector.size() == 0) {
        return;
    }
    spatialDetections.detections.resize(imgDetectionsVector.size());

    std::optional<std::vector<std::uint8_t>> optMaskData = imgDetections.getMaskData();

    const uint32_t lowerThreshold = config->globalLowerThreshold;
    const uint32_t upperThreshold = config->globalUpperThreshold;
    const SpatialLocationCalculatorAlgorithm calculationAlgorithm = config->globalCalculationAlgorithm;
    int stepSize = config->globalStepSize;
    if(stepSize == SpatialLocationCalculatorConfigData::AUTO) {
        if(calculationAlgorithm == SpatialLocationCalculatorAlgorithm::MODE || calculationAlgorithm == SpatialLocationCalculatorAlgorithm::MEDIAN) {
            stepSize = 2;
        } else {
            stepSize = 1;
        }
    }
    const int32_t keypointRadius = config->globalKeypointRadius;
    const bool calculateSpatialKeypoints = config->calculateSpatialKeypoints;
    const bool useSegmentation = config->useSegmentation && optMaskData.has_value();

    std::int32_t depthWidth = depthFrame->getWidth();
    std::int32_t depthHeight = depthFrame->getHeight();
    uint8_t* maskPtr = nullptr;
    if(useSegmentation) {
        logger->debug("Using segmentation for spatial image calculations.");
        std::size_t segmentationMaskWidth = imgDetections.getSegmentationMaskWidth();
        std::size_t segmentationMaskHeight = imgDetections.getSegmentationMaskHeight();
        std::optional<cv::Mat> optSeg = imgDetections.getCvSegmentationMask();
        if(!optSeg.has_value()) {
            logger->error("Segmentation mask expected but not available in ImgDetections.");
            throw;
        }
        // add if opencv support here
        cv::Mat seg = optSeg.value();
        if(static_cast<int>(segmentationMaskHeight) != depthHeight || static_cast<int>(segmentationMaskWidth) != depthWidth) {
            logger->debug("Segmentation mask size {}x{} does not match depth frame size {}x{}. Resizing to depth frame size.", segmentationMaskWidth, segmentationMaskHeight, depthWidth, depthHeight);
            cv::resize(seg, seg, cv::Size(depthWidth, depthHeight), 0, 0, cv::INTER_NEAREST);
        }
        maskPtr = seg.data;
    }

    const uint16_t* plane = (uint16_t*)depthFrame->data->getData().data();

    for(int i = 0; i < static_cast<int>(imgDetectionsVector.size()); i++) {
        const dai::ImgDetection& detection = imgDetectionsVector[i];
        dai::RotatedRect rotatedRect = detection.getBoundingBox();
        rotatedRect = rotatedRect.denormalize(depthWidth, depthHeight, true);

        const auto& outerPoints = rotatedRect.getOuterRect();
        unsigned int xstart = std::min(std::max(0, (int)outerPoints[0]), depthWidth - 1);
        unsigned int ystart = std::min(std::max(0, (int)outerPoints[1]), depthHeight - 1);
        unsigned int xend = std::min(depthWidth - 1, (int)outerPoints[2]);
        unsigned int yend = std::min(depthHeight - 1, (int)outerPoints[3]);

        std::uint32_t sum = 0;
        std::uint16_t min = 65535, max = 0;
        std::uint32_t counter = 0;

        std::vector<uint16_t> validPixels;
        for(unsigned int y = ystart; y < yend; y += stepSize) {
            for(unsigned int x = xstart; x < xend; x += stepSize) {
                int index = y * depthWidth + x;
                uint16_t px = plane[index];

                bool usePixel = true;
                if(useSegmentation) {
                    usePixel = (maskPtr) ? (maskPtr[index] == i) : true;
                }

                if(px > lowerThreshold && px < upperThreshold && usePixel) {
                    validPixels.push_back(px);
                    sum += px;
                    ++counter;
                    if(px < min) min = px;
                    if(px > max) max = px;
                }
            }
        }
        dai::SpatialImgDetection spatialDetection;
        if(calculateSpatialKeypoints) {
            logger->info("Calculating spatial keypoints for detection {}", i);
            std::vector<dai::SpatialKeypoint> spatialKeypoints;
            for(auto kp : detection.getKeypoints()) {
                int kpx = static_cast<int>(kp.imageCoordinates.x * depthWidth);
                int kpy = static_cast<int>(kp.imageCoordinates.y * depthHeight);

                int kpxstart = std::max(0, kpx - keypointRadius);
                int kpystart = std::max(0, kpy - keypointRadius);
                int kpxend = std::min(depthWidth - 1, kpx + keypointRadius);
                int kpyend = std::min(depthHeight - 1, kpy + keypointRadius);

                std::uint32_t kpSum = 0;
                std::uint16_t kpMin = 65535, max = 0;
                std::uint32_t kpCounter = 0;
                std::vector<uint16_t> kpValidPixels;
                for(int ky = kpystart; ky < kpyend; ky++) {
                    for(int kx = kpxstart; kx < kpxend; kx++) {
                        int index = ky * depthWidth + kx;
                        uint16_t px = plane[index];

                        bool usePixel = (std::hypot(kpx - kx, kpy - ky) <= keypointRadius);
                        if(usePixel && useSegmentation) {
                            usePixel = (maskPtr) ? (maskPtr[index] == i) : true;
                        }
                        if(px > lowerThreshold && px < upperThreshold && usePixel) {
                            kpValidPixels.push_back(px);
                            kpSum += px;
                            ++kpCounter;
                            if(px < kpMin) kpMin = px;
                            if(px > max) max = px;
                        }
                    }
                }

                float kpZ = calculateDepth(calculationAlgorithm, kpValidPixels, kpSum, kpCounter, kpMin, max);
                dai::Point2f kpCenter = dai::Point2f(kpx, kpy);
                dai::Point3f spatialCoordinates = calculateSpatialCoordinates(kpZ, imgDetections.transformation->getIntrinsicMatrix(), kpCenter);

                dai::SpatialKeypoint spatialKp;
                spatialKp.imageCoordinates = kp.imageCoordinates;
                spatialKp.confidence = kp.confidence;
                spatialKp.label = kp.label;
                spatialKp.spatialCoordinates = spatialCoordinates;

                spatialKeypoints.push_back(spatialKp);
            }
            spatialDetection.setKeypoints(spatialKeypoints);
        }

        float z = calculateDepth(calculationAlgorithm, validPixels, sum, counter, min, max);
        spatialDetection.setBoundingBox(rotatedRect);
        spatialDetection.label = detection.label;
        spatialDetection.confidence = detection.confidence;
        spatialDetection.labelName = detection.labelName;

        dai::Point3f spatialCoordinates = calculateSpatialCoordinates(z, imgDetections.transformation->getIntrinsicMatrix(), rotatedRect.center);
        logger->debug("Calculated spatial coordinates: {} {} {}", spatialCoordinates.x, spatialCoordinates.y, spatialCoordinates.z);

        spatialDetection.spatialCoordinates = spatialCoordinates;

        SpatialLocationCalculatorConfigData boundingBoxMapping;
        boundingBoxMapping.depthThresholds.lowerThreshold = lowerThreshold;
        boundingBoxMapping.depthThresholds.upperThreshold = upperThreshold;
        boundingBoxMapping.calculationAlgorithm = calculationAlgorithm;
        spatialDetection.boundingBoxMapping = boundingBoxMapping;
        spatialDetections.detections[i] = spatialDetection;
    }

    std::optional<std::vector<std::uint8_t>> maskData = imgDetections.getMaskData();
    if(maskData) {
        spatialDetections.setSegmentationMask(*maskData, imgDetections.getSegmentationMaskWidth(), imgDetections.getSegmentationMaskHeight());
    }
}

}  // namespace SpatialUtils
}  // namespace utilities
}  // namespace dai
