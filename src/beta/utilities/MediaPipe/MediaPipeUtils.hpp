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

#pragma once

#include <array>
#include <vector>

namespace dai {
namespace beta {
namespace utilities {
namespace MediaPipeUtils {

struct SSDAnchorOptions {
    int numLayers = 0;
    double minScale = 0.0;
    double maxScale = 0.0;
    int inputSizeHeight = 0;
    int inputSizeWidth = 0;
    double anchorOffsetX = 0.0;
    double anchorOffsetY = 0.0;
    std::vector<int> strides;
    std::vector<double> aspectRatios;
    bool reduceBoxesInLowestLayer = false;
    double interpolatedScaleAspectRatio = 0.0;
    bool fixedAnchorSize = false;
};

std::vector<std::array<double, 4>> generateAnchors(const SSDAnchorOptions& options);  // Mirrors MediaPipe ssd_anchors_calculator

std::vector<std::array<double, 4>> generateHandtrackerAnchors(int inputSizeWidth, int inputSizeHeight);

struct PalmDetections {                        // Ordered by descending confidence score
    std::vector<std::array<float, 4>> bboxes;  // [xCenter, yCenter, width, height], normalized to [0, 1]
    std::vector<float> scores;
    std::vector<float> angles;  // Degrees, rounded half to even
};

PalmDetections computeMediaPipePalmDetections(const std::vector<float>& bboxes,  // maxDet <= 0 means no limit; scale = model input size in pixels
                                              const std::vector<float>& scores,
                                              const std::vector<std::array<double, 4>>& anchors,
                                              float confThreshold,
                                              float iouThreshold,
                                              int maxDet,
                                              int scale);

}  // namespace MediaPipeUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
