// This file contains utility functions for decoding the output of the MediaPipe hand tracking
// (palm detection) model.
//
// This file contains code that is based on or directly taken from a public GitHub repository:
// https://github.com/geaxgx/depthai_hand_tracker
//
// Original code author(s): geaxgx
//
// License: MIT License
//
// Copyright (c) [2021] [geax]

#include "beta/utilities/MediaPipe/MediaPipeUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "beta/utilities/Detection/DetectionUtils.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace MediaPipeUtils {

namespace {

constexpr double PI = 3.14159265358979323846;

double calculateScale(double minScale, double maxScale, int strideIndex, int numStrides) {
    if(numStrides == 1) {
        return (minScale + maxScale) / 2.0;
    }
    return minScale + (maxScale - minScale) * strideIndex / (numStrides - 1);
}

/// Normalize an angle in radians to (-PI, PI].
double normalizeRadians(double angle) {
    return angle - 2.0 * PI * std::floor((angle + PI) / (2.0 * PI));
}

/// Round half to even (banker's rounding), matching np.round with 0 decimals.
double roundHalfToEven(double value) {
    const double floorValue = std::floor(value);
    const double diff = value - floorValue;
    if(diff > 0.5) {
        return floorValue + 1.0;
    }
    if(diff < 0.5) {
        return floorValue;
    }
    return std::fmod(floorValue, 2.0) == 0.0 ? floorValue : floorValue + 1.0;
}

/// Truncate toward zero, matching the Python int() conversion of a float.
long long truncateTowardZero(double value) {
    return static_cast<long long>(value);
}

}  // namespace

std::vector<std::array<double, 4>> generateAnchors(const SSDAnchorOptions& options) {
    // https://github.com/google/mediapipe/blob/master/mediapipe/calculators/tflite/ssd_anchors_calculator.cc
    std::vector<std::array<double, 4>> anchors;
    int layerId = 0;
    const int nStrides = static_cast<int>(options.strides.size());
    while(layerId < nStrides) {
        std::vector<double> anchorHeight;
        std::vector<double> anchorWidth;
        std::vector<double> aspectRatios;
        std::vector<double> scales;
        // For same strides, we merge the anchors in the same order.
        int lastSameStrideLayer = layerId;
        while(lastSameStrideLayer < nStrides && options.strides[lastSameStrideLayer] == options.strides[layerId]) {
            const double scale = calculateScale(options.minScale, options.maxScale, lastSameStrideLayer, nStrides);
            if(lastSameStrideLayer == 0 && options.reduceBoxesInLowestLayer) {
                // For first layer, it can be specified to use predefined anchors.
                aspectRatios.insert(aspectRatios.end(), {1.0, 2.0, 0.5});
                scales.insert(scales.end(), {0.1, scale, scale});
            } else {
                aspectRatios.insert(aspectRatios.end(), options.aspectRatios.begin(), options.aspectRatios.end());
                scales.insert(scales.end(), options.aspectRatios.size(), scale);
                if(options.interpolatedScaleAspectRatio > 0.0) {
                    const double scaleNext =
                        lastSameStrideLayer == nStrides - 1 ? 1.0 : calculateScale(options.minScale, options.maxScale, lastSameStrideLayer + 1, nStrides);
                    scales.push_back(std::sqrt(scale * scaleNext));
                    aspectRatios.push_back(options.interpolatedScaleAspectRatio);
                }
            }
            lastSameStrideLayer++;
        }

        for(std::size_t i = 0; i < aspectRatios.size(); i++) {
            const double ratioSqrt = std::sqrt(aspectRatios[i]);
            anchorHeight.push_back(scales[i] / ratioSqrt);
            anchorWidth.push_back(scales[i] * ratioSqrt);
        }

        const int stride = options.strides[layerId];
        const int featureMapHeight = static_cast<int>(std::ceil(static_cast<double>(options.inputSizeHeight) / stride));
        const int featureMapWidth = static_cast<int>(std::ceil(static_cast<double>(options.inputSizeWidth) / stride));

        for(int y = 0; y < featureMapHeight; y++) {
            for(int x = 0; x < featureMapWidth; x++) {
                for(std::size_t anchorId = 0; anchorId < anchorHeight.size(); anchorId++) {
                    const double xCenter = (x + options.anchorOffsetX) / featureMapWidth;
                    const double yCenter = (y + options.anchorOffsetY) / featureMapHeight;
                    if(options.fixedAnchorSize) {
                        anchors.push_back({xCenter, yCenter, 1.0, 1.0});
                    } else {
                        anchors.push_back({xCenter, yCenter, anchorWidth[anchorId], anchorHeight[anchorId]});
                    }
                }
            }
        }

        layerId = lastSameStrideLayer;
    }
    return anchors;
}

std::vector<std::array<double, 4>> generateHandtrackerAnchors(int inputSizeWidth, int inputSizeHeight) {
    // https://github.com/google/mediapipe/blob/master/mediapipe/modules/palm_detection/palm_detection_cpu.pbtxt
    SSDAnchorOptions anchorOptions;
    anchorOptions.numLayers = 4;
    anchorOptions.minScale = 0.1484375;
    anchorOptions.maxScale = 0.75;
    anchorOptions.inputSizeHeight = inputSizeHeight;
    anchorOptions.inputSizeWidth = inputSizeWidth;
    anchorOptions.anchorOffsetX = 0.5;
    anchorOptions.anchorOffsetY = 0.5;
    anchorOptions.strides = {8, 16, 16, 16};
    anchorOptions.aspectRatios = {1.0};
    anchorOptions.reduceBoxesInLowestLayer = false;
    anchorOptions.interpolatedScaleAspectRatio = 1.0;
    anchorOptions.fixedAnchorSize = true;
    return generateAnchors(anchorOptions);
}

PalmDetections computeMediaPipePalmDetections(const std::vector<float>& bboxes,
                                              const std::vector<float>& scores,
                                              const std::vector<std::array<double, 4>>& anchors,
                                              float confThreshold,
                                              float iouThreshold,
                                              int maxDet,
                                              int scale) {
    PalmDetections result;

    // Sigmoid on the raw scores, then a strict greater-than threshold filter, mirroring the
    // source decode_bboxes.
    const std::size_t numScores = scores.size();
    std::vector<float> sigmoidScores(numScores);
    for(std::size_t i = 0; i < numScores; i++) {
        sigmoidScores[i] = 1.0f / (1.0f + std::exp(-scores[i]));
    }
    std::vector<std::size_t> selectedIndices;
    for(std::size_t i = 0; i < numScores; i++) {
        if(sigmoidScores[i] > confThreshold) {
            selectedIndices.push_back(i);
        }
    }
    if(selectedIndices.empty()) {
        return result;
    }

    // The detection mask indexes both the bounding box rows and the anchors; a mismatching
    // anchor count means the anchors were generated for a different model input size.
    const std::size_t numBoxes = bboxes.size() / 18;
    if(numBoxes != numScores || anchors.size() != numScores) {
        throw std::runtime_error("Wrong parser scale set. Please use setScale method to set different scale according to model dimensions (e.g. 128).");
    }

    // Decode the kept rows against the SSD anchors. Each row holds the bounding box
    // (cx, cy, w, h) followed by 7 palm keypoint (x, y) pairs. Column j is scaled by the anchor
    // width/height (alternating) over the input scale and offset by the anchor x/y center; then
    // the box size is offset back by the anchor center and the box center is converted to the
    // top-left corner. Both x and y are shifted by half the box height, mirroring the source's
    // det_bboxes[:, 0:2] -= det_bboxes[:, 3:4] * 0.5 broadcasting.
    struct HandRegion {
        float pdScore = 0.0f;
        std::array<double, 18> values{};
    };
    std::vector<HandRegion> regions;
    for(const std::size_t idx : selectedIndices) {
        std::array<double, 18> decoded{};
        for(std::size_t j = 0; j < 18; j++) {
            decoded[j] = static_cast<double>(bboxes[idx * 18 + j]) * anchors[idx][2 + (j % 2)] / static_cast<double>(scale) + anchors[idx][j % 2];
        }
        decoded[2] -= anchors[idx][0];
        decoded[3] -= anchors[idx][1];
        decoded[0] -= decoded[3] * 0.5;
        decoded[1] -= decoded[3] * 0.5;
        // Decoded detection boxes could have negative values for width/height due to model
        // prediction. Filter out those boxes.
        if(decoded[2] < 0.0 || decoded[3] < 0.0) {
            continue;
        }
        regions.push_back(HandRegion{sigmoidScores[idx], decoded});
    }

    std::vector<std::array<double, 4>> bboxList;
    std::vector<std::array<double, 4>> nmsBboxList;
    std::vector<float> scoreList;
    std::vector<double> angleList;
    for(const auto& region : regions) {
        const auto& decoded = region.values;

        // detections_to_rect: the rectangle is rotated such that the line connecting the wrist
        // keypoint (index 0) and the MCP of the middle finger (index 2) is aligned with the
        // y-axis of the rectangle.
        const double rectWidth = decoded[2];
        const double rectHeight = decoded[3];
        const double rectXCenter = decoded[0] + rectWidth / 2.0;
        const double rectYCenter = decoded[1] + rectHeight / 2.0;
        const double x0 = decoded[4];  // wrist center
        const double y0 = decoded[5];
        const double x1 = decoded[8];  // middle finger
        const double y1 = decoded[9];
        const double targetAngle = PI * 0.5;  // 90 = pi/2
        const double rotation = normalizeRadians(targetAngle - std::atan2(-(y1 - y0), x1 - x0));

        // rect_transformation with no_shift=True: unit scale and no shift; the rectangle is
        // converted to pixels in the squared (scale x scale) image and expanded to a square over
        // its long side.
        const double xCenterA = rectXCenter * scale;
        const double yCenterA = rectYCenter * scale;
        const double longSide = std::max(rectWidth * scale, rectHeight * scale);
        const double widthA = longSide;
        const double heightA = longSide;

        // rotated_rect_to_points, including the source's truncating int() conversions.
        const double b = std::cos(rotation) * 0.5;
        const double a = std::sin(rotation) * 0.5;
        const double p0xF = xCenterA - a * heightA - b * widthA;
        const double p0yF = yCenterA + b * heightA - a * widthA;
        const double p1xF = xCenterA + a * heightA - b * widthA;
        const double p1yF = yCenterA - b * heightA - a * widthA;
        const long long p2x = truncateTowardZero(2.0 * xCenterA - p0xF);
        const long long p2y = truncateTowardZero(2.0 * yCenterA - p0yF);
        const long long p3x = truncateTowardZero(2.0 * xCenterA - p1xF);
        const long long p3y = truncateTowardZero(2.0 * yCenterA - p1yF);
        const long long p0x = truncateTowardZero(p0xF);
        const long long p0y = truncateTowardZero(p0yF);
        const long long p1x = truncateTowardZero(p1xF);
        const long long p1y = truncateTowardZero(p1yF);

        // The rectangle's angle, center and size are derived from the integer corner points.
        const double xDist = static_cast<double>(p3x - p0x);
        const double yDist = static_cast<double>(p3y - p0y);
        const double angle = std::atan2(yDist, xDist) * 180.0 / PI;
        const double xCenter = static_cast<double>(p0x + p1x + p2x + p3x) / 4.0;
        const double yCenter = static_cast<double>(p0y + p1y + p2y + p3y) / 4.0;
        const double width = std::sqrt(xDist * xDist + yDist * yDist);
        const double heightXDist = static_cast<double>(p0x - p1x);
        const double heightYDist = static_cast<double>(p0y - p1y);
        const double height = std::sqrt(heightXDist * heightXDist + heightYDist * heightYDist);
        const double xMin = xCenter - width / 2.0;
        const double yMin = yCenter - height / 2.0;

        bboxList.push_back({xCenter, yCenter, width, height});
        nmsBboxList.push_back({xMin, yMin, width, height});
        angleList.push_back(angle);
        scoreList.push_back(region.pdScore);
    }

    if(bboxList.empty()) {
        return result;
    }

    const std::vector<int> keptIndices = DetectionUtils::nmsBoxes(nmsBboxList, scoreList, confThreshold, iouThreshold, maxDet);
    if(keptIndices.empty()) {
        return result;
    }

    result.bboxes.reserve(keptIndices.size());
    result.scores.reserve(keptIndices.size());
    result.angles.reserve(keptIndices.size());
    for(const int idx : keptIndices) {
        // The kept boxes are cast to float32, normalized by the scale and clipped to [0, 1]; the
        // kept angles are rounded to 0 decimals (half to even).
        std::array<float, 4> bbox{};
        for(std::size_t k = 0; k < 4; k++) {
            const float normalized = static_cast<float>(bboxList[idx][k]) / static_cast<float>(scale);
            bbox[k] = std::clamp(normalized, 0.0f, 1.0f);
        }
        result.bboxes.push_back(bbox);
        result.scores.push_back(scoreList[idx]);
        result.angles.push_back(static_cast<float>(roundHalfToEven(angleList[idx])));
    }
    return result;
}

}  // namespace MediaPipeUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
