#pragma once

#include <spdlog/async_logger.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "FeatureTracker.hpp"
#include "FeatureTrackerAccel.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {
namespace gate {

template <class IntegerType>
float qToFloat(IntegerType q, int fractional_bits) {
    return ((float)q) * std::pow(2, -fractional_bits);
}

/**
 * Detect features in previous frame and add those which are:
 * ----- not too close to any of the already tracked features
 * ----- satisfy the Harris Corner detection score threshold
 *
 **/
void addMoreFeaturesToPreviousFrame(std::map<TrackedPoint, uint32_t>& detectedFeaturesInPreviousFrame,
                                    const CornerDetector& cornerDetector,
                                    uint32_t limit,
                                    uint32_t& featureId,
                                    std::shared_ptr<spdlog::async_logger> logger,
                                    int maxNumberOfFeatures,
                                    float minimumDistanceBetweenFeatures) {
    std::map<TrackedPoint, uint32_t> valuesToBeAdded{};
    const FeaturePoint* points = cornerDetector.points();
    for(size_t i = 0; i < cornerDetector.numPoints(); i++) {
        if(detectedFeaturesInPreviousFrame.size() + valuesToBeAdded.size() > 8 * maxNumberOfFeatures) {
            logger->trace("Too many features detected!");
            logger->trace("maxNumberOfFeatures: {}", maxNumberOfFeatures);
            return;
        }

        auto current = points[i];
        uint32_t harrisScore = current.score & FeaturePoint::HARRIS_SCORE_MASK;  // last 28 bits

        if(harrisScore > limit) {
            TrackedPoint trackedPoint(current.x, current.y, harrisScore, 0, 0);

            // First check if it is already in there
            if(detectedFeaturesInPreviousFrame.find(trackedPoint) == detectedFeaturesInPreviousFrame.end()) {
                bool addThisPoint = true;
                for(const auto& [current, second] : detectedFeaturesInPreviousFrame) {
                    TrackedPoint other(current.x, current.y, 0, 0, 0);

                    if(trackedPoint.distance(other) < minimumDistanceBetweenFeatures) {
                        addThisPoint = false;
                        break;
                    }
                }
                if(addThisPoint) {
                    // Only add it if it is far enough of all existing points
                    featureId++;
                    valuesToBeAdded[trackedPoint] = featureId;
                }
            }
        }
    }

    for(const auto& [first, second] : valuesToBeAdded) {
        detectedFeaturesInPreviousFrame[first] = second;
    }
}

/**
 * Detect features in current frame and accept them all
 **/
void detectFeaturesInCurrentFrame(const CornerDetector& cornerDetector,
                                  const uint32_t limit,
                                  uint32_t& featureId,
                                  const std::shared_ptr<spdlog::async_logger> logger,
                                  const int maxNumberOfFeatures,
                                  std::shared_ptr<dai::TrackedFeatures> outFeatures) {
    outFeatures->trackedFeatures.reserve(maxNumberOfFeatures);

    const FeaturePoint* points = cornerDetector.points();

    for(size_t i = 0; i < cornerDetector.numPoints(); i++) {
        const auto& current = points[i];

        uint32_t harrisScore = current.score & FeaturePoint::HARRIS_SCORE_MASK;  // last 28 bits

        if(harrisScore > limit) {
            dai::TrackedFeature feature{
                {static_cast<float>(current.x), static_cast<float>(current.y)}, featureId, 0, static_cast<float>(current.score), static_cast<float>(0)};

            outFeatures->trackedFeatures.push_back(feature);
            featureId++;

            if(outFeatures->trackedFeatures.size() >= maxNumberOfFeatures) {
                logger->trace("Too many features detected DCM!");
                return;
            }
        }
    }
}

/**
 *  Tracking and reidentification using motion estimation between 2 consecutive frames
 *  From the point of feature pixel movement, we both scenarios: Feature that moved and features that didn't move
 **/
std::map<TrackedPoint, uint32_t> reidentifyUsingDenseOpticalFlow(const std::map<TrackedPoint, uint32_t>& detectedFeaturesInPreviousFrame,
                                                                 const OpticalFlowAccel& opticalFlow,
                                                                 const CornerDetector& cornerDetector,
                                                                 uint32_t width,
                                                                 uint32_t height,
                                                                 std::shared_ptr<spdlog::async_logger> logger,
                                                                 int maxNumberOfFeatures,
                                                                 int confidenceThreshold) {
    std::map<TrackedPoint, uint32_t> reidintifiedFeaturesInCurrentFrame;

    const int16_t* pFwdMvMap = opticalFlow.forwardMotionVectors();
    const int16_t* pBwdMvMap = opticalFlow.backwardMotionVectors();
    const uint8_t* pConfMap = opticalFlow.confidenceMap();

    for(auto [current, second] : detectedFeaturesInPreviousFrame) {
        uint32_t positionConfidence = current.y * width + current.x;
        uint32_t positionX = positionConfidence * 2;
        uint32_t positionY = positionX + 1;

        uint8_t confidence = pConfMap[positionConfidence];

        int16_t xMoveFwd = pFwdMvMap[positionX];
        int16_t yMoveFwd = pFwdMvMap[positionY];
        int16_t xMoveFwdShift = xMoveFwd >> 4;
        int16_t yMoveFwdShift = yMoveFwd >> 4;

        int16_t xMoveBwd = pBwdMvMap[positionX];
        int16_t yMoveBwd = pBwdMvMap[positionY];
        int16_t xMoveBwdShift = xMoveBwd >> 4;
        int16_t yMoveBwdShift = yMoveBwd >> 4;

        if(confidence >= confidenceThreshold) {
            if((std::abs(xMoveFwdShift + xMoveBwdShift) <= 5) and (std::abs(yMoveFwdShift + yMoveBwdShift) <= 5)) {
                TrackedPoint trackedPoint;

                // Move to signed arithmetic
                int x = current.x;
                int y = current.y;

                x += xMoveFwdShift;
                y += yMoveFwdShift;

                x = std::min(x, static_cast<int>(width - 1));
                y = std::min(y, static_cast<int>(height - 1));

                x = std::max(x, 0);
                y = std::max(y, 0);

                trackedPoint.x = x;
                trackedPoint.y = y;
                trackedPoint.age = current.age + 1;
                trackedPoint.score = current.score;
                trackedPoint.confidence = confidence;

                reidintifiedFeaturesInCurrentFrame[trackedPoint] = second;

                if(reidintifiedFeaturesInCurrentFrame.size() >= maxNumberOfFeatures) {
                    return reidintifiedFeaturesInCurrentFrame;
                }
            }
        }
    }

    return reidintifiedFeaturesInCurrentFrame;
}

std::vector<DescriptorSearchPosition> convertFpxToDcmFeaturePositions(std::vector<dai::TrackedFeature>& trackedFeatures,
                                                                      bool lpfOn,
                                                                      uint16_t height,
                                                                      uint16_t width) {
    // The descriptor calculator handles at most MAX_POINTS positions per run
    const size_t numPositions = std::min<size_t>(trackedFeatures.size(), DescriptorCalculator::MAX_POINTS);
    std::vector<DescriptorSearchPosition> dcmSearchPositions(numPositions);

    uint16_t offset = 18u;
    if(lpfOn) {
        offset = 20u;
    }
    uint16_t unit = 1u;

    for(size_t i = 0; i < numPositions; i++) {
        dcmSearchPositions[i].x = std::min(static_cast<uint16_t>(trackedFeatures[i].position.x) + offset, height - unit);
        dcmSearchPositions[i].y = std::min(static_cast<uint16_t>(trackedFeatures[i].position.y) + offset, width - unit);
    }

    return dcmSearchPositions;
}

void calculateOrbDescriptors(std::shared_ptr<dai::ImgFrame> inImageCurrentFrame,
                             DescriptorCalculator& descriptorCalculator,
                             std::shared_ptr<dai::TrackedFeatures> outFeatures) {
    std::vector<DescriptorSearchPosition> positionsInCurrentFrame = convertFpxToDcmFeaturePositions(outFeatures->trackedFeatures,
                                                                                                    descriptorCalculator.descriptorLowPassFilter(),
                                                                                                    static_cast<uint16_t>(inImageCurrentFrame->getHeight()),
                                                                                                    static_cast<uint16_t>(inImageCurrentFrame->getWidth()));

    // Set the points for DCM
    descriptorCalculator.setNumPoints(positionsInCurrentFrame.size());
    // Reconfigure DCM
    descriptorCalculator.Config();

    descriptorCalculator.Run(inImageCurrentFrame, positionsInCurrentFrame);

    const uint8_t* descData = descriptorCalculator.descriptors();

    // Descriptors exist only for the positions that were computed
    const size_t numDescriptors = std::min(outFeatures->trackedFeatures.size(), positionsInCurrentFrame.size());
    for(size_t i = 0; i < numDescriptors; i++) {
        for(size_t j = 0; j < dai::gate::FeatureTracker::DESCRIPTOR_SIZE; j++) {
            outFeatures->trackedFeatures[i].descriptor[j] = *descData;
            descData++;
        }
    }
}

void extractFeatures(std::map<TrackedPoint, uint32_t>& detectedFeatures, std::shared_ptr<dai::TrackedFeatures> outFeatures) {
    outFeatures->trackedFeatures.reserve(detectedFeatures.size());

    for(auto& [trackedPoint, featureId] : detectedFeatures) {
        dai::TrackedFeature feature{{static_cast<float>(trackedPoint.x), static_cast<float>(trackedPoint.y)},
                                    featureId,
                                    trackedPoint.age,
                                    static_cast<float>(trackedPoint.score),
                                    static_cast<float>(trackedPoint.confidence)};

        outFeatures->trackedFeatures.push_back(feature);
    }
}

void printConfig(const dai::FeatureTrackerConfig& latestConfig, std::shared_ptr<spdlog::async_logger> logger) {
    logger->debug("InitialConfig for motionEstimator enable: {}", static_cast<int>(latestConfig.motionEstimator.enable));
    logger->debug("InitialConfig for cornerDetector type: {}", static_cast<int>(latestConfig.cornerDetector.type));
    logger->debug("InitialConfig for cornerDetector numMaxFeatures: {}", latestConfig.cornerDetector.numMaxFeatures);
    logger->debug("InitialConfig for cornerDetector thresholds.harrisScore to: {}", latestConfig.cornerDetector.thresholds.initialValue);
    // logger->error("InitialConfig for cornerDetector robustness: {}", latestConfig.cornerDetector.robustness);
    // logger->error("InitialConfig for enableDescriptorCalculation: {}", latestConfig.cornerDetector.enableDescriptorCalculation);
}

}  // namespace gate
}  // namespace dai
