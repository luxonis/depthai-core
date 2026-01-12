#include "SpatialUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include "common/Point2f.hpp"
#include "common/RotatedRect.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/SpatialKeypoint.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace utilities {
namespace SpatialUtils {

DepthStats::DepthStats(std::uint32_t lowerThreshold, std::uint32_t upperThreshold, std::uint32_t maxNumPixels)
    : lowerThreshold(lowerThreshold), upperThreshold(upperThreshold) {
    validPixels.reserve(maxNumPixels);
}

void DepthStats::addPixel(uint16_t px) {
    if(px > lowerThreshold && px < upperThreshold) {
        validPixels.push_back(px);
        sum += static_cast<std::double_t>(px);
        ++counter;
        if(px < min) min = static_cast<std::uint32_t>(px);
        if(px > max) max = static_cast<std::uint32_t>(px);
    }
}

float DepthStats::calculateDepth(SpatialLocationCalculatorAlgorithm algo) {
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

dai::Point3f calculateSpatialCoordinates(float z, const std::array<std::array<float, 3>, 3>& intrinsicMatrix, dai::Point2f center) {
    float fx = intrinsicMatrix[0][0];
    float fy = intrinsicMatrix[1][1];
    float cx = intrinsicMatrix[0][2];
    float cy = intrinsicMatrix[1][2];

    float x = (center.x - cx) * z / fx;
    float y = (center.y - cy) * z / fy;

    return dai::Point3f(x, y, z);
}

static inline dai::Point2f clampPoint(dai::Point2f point, int width, int height) {
    float x = std::min(std::max(0.0f, point.x), static_cast<float>(width));
    float y = std::min(std::max(0.0f, point.y), static_cast<float>(height));
    return dai::Point2f(x, y);
}

static inline std::array<int, 4> clampRectCorners(dai::Point2f topLeft, dai::Point2f bottomRight, int width, int height) {
    dai::Point2f clampedTopLeft = clampPoint(topLeft, width - 1, height - 1);  // -1 to ensure start is within bounds
    dai::Point2f clampedBottomRight = clampPoint(bottomRight, width, height);

    int xstart = std::min(std::max(0, static_cast<int>(clampedTopLeft.x)), width);
    int ystart = std::min(std::max(0, static_cast<int>(clampedTopLeft.y)), height);
    int xend = std::min(std::max(0, static_cast<int>(clampedBottomRight.x)), width);
    int yend = std::min(std::max(0, static_cast<int>(clampedBottomRight.y)), height);

    return {xstart, ystart, xend, yend};
}

static inline int getStepSize(int stepSize, SpatialLocationCalculatorAlgorithm algo) {
    if(stepSize == SpatialLocationCalculatorConfigData::AUTO) {
        if(algo == SpatialLocationCalculatorAlgorithm::MODE || algo == SpatialLocationCalculatorAlgorithm::MEDIAN) {
            return 2;
        } else {
            return 1;
        }
    }

    return stepSize;
}

void computeSpatialData(std::shared_ptr<dai::ImgFrame> depthFrame,
                        const std::vector<dai::SpatialLocationCalculatorConfigData>& configDataVec,
                        std::vector<dai::SpatialLocations>& spatialLocations,
                        std::shared_ptr<spdlog::async_logger> logger) {
    if(configDataVec.empty()) {
        logger->debug("Empty config vector!");
        return;
    }

    if(!depthFrame->transformation.isValid()) {
        throw std::runtime_error("DepthFrame has invalid Image Transformations. Cannot get intrinsic matrix.");
    }

    spatialLocations.resize(configDataVec.size());
    std::int32_t depthWidth = depthFrame->getWidth();
    std::int32_t depthHeight = depthFrame->getHeight();
    for(size_t i = 0; i < configDataVec.size(); i++) {
        const auto& cfg = configDataVec[i];
        const auto& roi = cfg.roi;
        float maxWidthSize = 1;
        float maxHeightSize = 1;
        const SpatialLocationCalculatorAlgorithm calcAlgo = cfg.calculationAlgorithm;
        const int stepSize = getStepSize(cfg.stepSize, calcAlgo);
        if(stepSize >= depthWidth / 2 || stepSize >= depthHeight / 2) {
            auto message = fmt::format("Step size {} is too large for depth frame size {}x{}. It must be less than half of the depth frame dimensions.",
                                       stepSize,
                                       depthWidth,
                                       depthHeight);
            throw std::invalid_argument(message);
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

        dai::Rect denormRoi = roi.denormalize(depthWidth, depthHeight);
        auto [xstart, ystart, xend, yend] = clampRectCorners(denormRoi.topLeft(), denormRoi.bottomRight(), depthWidth, depthHeight);

        std::uint32_t maxNumPixels = ((xend - xstart + stepSize - 1) / stepSize) * ((yend - ystart + stepSize - 1) / stepSize);
        DepthStats depthStats(cfg.depthThresholds.lowerThreshold, cfg.depthThresholds.upperThreshold, maxNumPixels);

        const uint16_t* plane = (uint16_t*)depthFrame->data->getData().data();
        for(int y = ystart; y < yend; y += stepSize) {
            for(int x = xstart; x < xend; x += stepSize) {
                uint16_t px = plane[y * depthWidth + x];
                depthStats.addPixel(px);
            }
        }

        SpatialLocations spatialData = {};
        spatialData.depthAverage = depthStats.calculateDepth(SpatialLocationCalculatorAlgorithm::AVERAGE);
        spatialData.depthMin = depthStats.min;
        spatialData.depthMax = depthStats.max;
        spatialData.config = cfg;
        spatialData.depthAveragePixelCount = depthStats.counter;

        float z = depthStats.calculateDepth(calcAlgo);

        float roiCx = denormRoi.x + denormRoi.width / 2.0f;
        float roiCy = denormRoi.y + denormRoi.height / 2.0f;

        dai::Point3f spatialCoordinates = calculateSpatialCoordinates(z, depthFrame->transformation.getIntrinsicMatrix(), dai::Point2f(roiCx, roiCy));
        logger->trace("Calculated spatial coordinates: {} {} {}", spatialCoordinates.x, spatialCoordinates.y, spatialCoordinates.z);

        spatialData.spatialCoordinates = spatialCoordinates;
        spatialLocations[i] = spatialData;
    }
}

void computeSpatialDetections(const dai::ImgFrame& depthFrame,
                              const SpatialLocationCalculatorConfig& config,
                              const dai::ImgDetections& imgDetections,
                              dai::SpatialImgDetections& spatialDetections,
                              std::shared_ptr<spdlog::async_logger> logger) {
    if(!imgDetections.transformation.has_value()) {
        throw std::runtime_error("No transformation set on ImgDetections. Cannot compute spatial coordinates.");
    }

    if(!imgDetections.transformation.value().isValid()) {
        throw std::runtime_error("ImgDetections has invalid Image Transformations. Cannot map segmentation mask to depth frame.");
    }

    if(!depthFrame.transformation.isValid()) {
        throw std::runtime_error("DepthFrame has invalid Image Transformations. Cannot map segmentation mask to depth frame.");
    }

    const dai::ImgTransformation* depthTransformation = &depthFrame.transformation;
    const dai::ImgTransformation* detectionsTransformation = &imgDetections.transformation.value();
    const bool areAligned = detectionsTransformation->isAlignedTo(*depthTransformation);

    if(!areAligned) {
        logger->warn(
            "DepthFrame and ImgDetections transformations are not aligned and processing may be slowed down due to need for remapping points. Consider using "
            "ImageAlign node beforehand.");
    }

    std::vector<dai::ImgDetection> imgDetectionsVector = imgDetections.detections;
    if(imgDetectionsVector.size() == 0) {
        return;
    }
    spatialDetections.detections.resize(imgDetectionsVector.size());
    span<const std::uint8_t> maskSpan = imgDetections.getData();

    const uint32_t lowerThreshold = config.globalLowerThreshold;
    const uint32_t upperThreshold = config.globalUpperThreshold;
    const SpatialLocationCalculatorAlgorithm calculationAlgorithm = config.globalCalculationAlgorithm;
    const int stepSize = getStepSize(config.globalStepSize, calculationAlgorithm);
    const int32_t keypointRadius = config.globalKeypointRadius;
    const bool calculateSpatialKeypoints = config.calculateSpatialKeypoints;
    const bool useSegmentation = config.useSegmentation && !maskSpan.empty();
    const bool passthroughSegmentation = config.segmentationPassthrough && !maskSpan.empty();

    const std::int32_t depthWidth = depthFrame.getWidth();
    const std::int32_t depthHeight = depthFrame.getHeight();
    const uint8_t* maskPtr = nullptr;
    std::size_t segmentationMaskWidth = 0;
    std::size_t segmentationMaskHeight = 0;

    if(stepSize >= depthWidth / 2 || stepSize >= depthHeight / 2) {
        auto message = fmt::format("Step size {} is too large for depth frame size {}x{}. It must be less than half of the depth frame dimensions.",
                                   stepSize,
                                   depthWidth,
                                   depthHeight);
        throw std::invalid_argument(message);
    }

    if(useSegmentation && imgDetections.getSegmentationMaskWidth() > 0 && imgDetections.getSegmentationMaskHeight() > 0) {
        maskPtr = maskSpan.data();
    }

    const uint16_t* plane = (uint16_t*)depthFrame.data->getData().data();

    for(int i = 0; i < static_cast<int>(imgDetectionsVector.size()); i++) {
        const dai::ImgDetection& detection = imgDetectionsVector[i];
        const dai::RotatedRect detectionBoundingBox = detection.getBoundingBox();
        dai::RotatedRect depthAlignedRect = detectionBoundingBox;
        if(!areAligned) depthAlignedRect = detectionsTransformation->remapRectTo(*depthTransformation, depthAlignedRect);

        dai::RotatedRect denormalizedRect = depthAlignedRect.denormalize(depthWidth, depthHeight);

        const auto& outerPoints = denormalizedRect.getOuterRect();
        auto [xstart, ystart, xend, yend] =
            clampRectCorners(dai::Point2f{outerPoints[0], outerPoints[1]}, dai::Point2f{outerPoints[2], outerPoints[3]}, depthWidth, depthHeight);

        std::uint32_t maxNumPixels = ((xend - xstart + stepSize - 1) / stepSize) * ((yend - ystart + stepSize - 1) / stepSize);
        DepthStats depthStats(lowerThreshold, upperThreshold, maxNumPixels);

        for(int y = ystart; y < yend; y += stepSize) {
            for(int x = xstart; x < xend; x += stepSize) {
                int index = y * depthWidth + x;
                uint16_t px = plane[index];

                bool usePixel = true;
                if(useSegmentation) {
                    dai::Point2f maskPoint{static_cast<float>(x), static_cast<float>(y)};
                    if(!areAligned) {
                        maskPoint = depthTransformation->remapPointTo(*detectionsTransformation, maskPoint);
                        maskPoint = clampPoint(maskPoint, segmentationMaskWidth - 1, segmentationMaskHeight - 1);
                    }
                    int maskIndex = static_cast<int>(maskPoint.y) * segmentationMaskWidth + static_cast<int>(maskPoint.x);
                    usePixel = (maskPtr) ? (maskPtr[maskIndex] == i) : true;
                }

                if(usePixel) {
                    depthStats.addPixel(px);
                }
            }
        }

        dai::SpatialImgDetection spatialDetection;
        if(calculateSpatialKeypoints) {
            std::vector<dai::SpatialKeypoint> spatialKeypoints;
            for(auto kp : detection.getKeypoints()) {
                dai::Point2f mappedKp = dai::Point2f{kp.imageCoordinates.x, kp.imageCoordinates.y};
                if(!areAligned) mappedKp = detectionsTransformation->remapPointTo(*depthTransformation, mappedKp);

                float kpx = mappedKp.x * depthWidth;
                float kpy = mappedKp.y * depthHeight;
                auto [kpxstart, kpystart, kpxend, kpyend] = clampRectCorners(dai::Point2f{kpx - keypointRadius, kpy - keypointRadius},
                                                                             dai::Point2f{kpx + keypointRadius, kpy + keypointRadius},
                                                                             depthWidth,
                                                                             depthHeight);

                DepthStats kpDepthStats(lowerThreshold, upperThreshold, static_cast<std::uint32_t>((kpxend - kpxstart) * (kpyend - kpystart)));
                for(int ky = kpystart; ky < kpyend; ky++) {
                    for(int kx = kpxstart; kx < kpxend; kx++) {
                        int index = ky * depthWidth + kx;
                        uint16_t px = plane[index];

                        bool usePixel = (std::hypot(kpx - kx, kpy - ky) <= keypointRadius);
                        if(usePixel && useSegmentation) {
                            dai::Point2f maskKpPoint{static_cast<float>(kx), static_cast<float>(ky)};
                            if(!areAligned) {
                                maskKpPoint = depthTransformation->remapPointTo(*detectionsTransformation, maskKpPoint);
                                maskKpPoint = clampPoint(maskKpPoint, segmentationMaskWidth - 1, segmentationMaskHeight - 1);
                            }
                            int maskIndex = static_cast<int>(maskKpPoint.y) * segmentationMaskWidth + static_cast<int>(maskKpPoint.x);
                            usePixel = (maskPtr) ? (maskPtr[maskIndex] == i) : true;
                        }
                        if(usePixel) {
                            kpDepthStats.addPixel(px);
                        }
                    }
                }

                float kpZ = kpDepthStats.calculateDepth(calculationAlgorithm);
                dai::Point2f kpPointCoordinates{kpx, kpy};
                if(!areAligned) {
                    kpPointCoordinates = depthTransformation->remapPointFrom(*detectionsTransformation, kpPointCoordinates);
                }
                dai::Point3f spatialCoordinates = calculateSpatialCoordinates(kpZ, detectionsTransformation->getIntrinsicMatrix(), kpPointCoordinates);

                dai::SpatialKeypoint spatialKp;
                spatialKp.imageCoordinates = kp.imageCoordinates;
                spatialKp.confidence = kp.confidence;
                spatialKp.label = kp.label;
                spatialKp.spatialCoordinates = spatialCoordinates;

                spatialKeypoints.push_back(spatialKp);
            }
            spatialDetection.setKeypoints(spatialKeypoints);
        }

        float z = depthStats.calculateDepth(calculationAlgorithm);
        spatialDetection.setBoundingBox(detectionBoundingBox);
        spatialDetection.label = detection.label;
        spatialDetection.confidence = detection.confidence;
        spatialDetection.labelName = detection.labelName;

        // Remap back to original detection coordinates
        if(!areAligned) denormalizedRect = depthTransformation->remapRectTo(*detectionsTransformation, denormalizedRect);

        dai::Point3f spatialCoordinates = calculateSpatialCoordinates(z, detectionsTransformation->getIntrinsicMatrix(), denormalizedRect.center);
        logger->trace("Calculated spatial coordinates: {} {} {}", spatialCoordinates.x, spatialCoordinates.y, spatialCoordinates.z);

        spatialDetection.spatialCoordinates = spatialCoordinates;

        SpatialLocationCalculatorConfigData boundingBoxMapping;
        boundingBoxMapping.depthThresholds.lowerThreshold = lowerThreshold;
        boundingBoxMapping.depthThresholds.upperThreshold = upperThreshold;
        boundingBoxMapping.calculationAlgorithm = calculationAlgorithm;
        spatialDetection.boundingBoxMapping = boundingBoxMapping;
        spatialDetections.detections[i] = spatialDetection;
    }

    if(passthroughSegmentation) {
        spatialDetections.data = imgDetections.data;
        spatialDetections.segmentationMaskWidth = imgDetections.getSegmentationMaskWidth();
        spatialDetections.segmentationMaskHeight = imgDetections.getSegmentationMaskHeight();
    }
}

}  // namespace SpatialUtils
}  // namespace utilities
}  // namespace dai
