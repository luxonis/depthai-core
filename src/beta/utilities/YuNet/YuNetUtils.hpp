// This file contains utility functions for decoding the output of the YuNet face detection
// model.
//
// This file contains code that is based on or directly taken from a public GitHub repository:
// https://github.com/Kazuhito00/YuNet-ONNX-TFLite-Sample
//
// Original code author(s): Kazuhito00
//
// License: Apache License 2.0
//
// Copyright (c) [2021] [Kazuhito00]

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dai {
namespace beta {
namespace utilities {
namespace YuNetUtils {

constexpr std::size_t NUM_KEYPOINTS = 5;  // Fixed by the YuNet output contract, not a per-model value

constexpr std::size_t LOC_VALUES_PER_ANCHOR = 4 + 2 * NUM_KEYPOINTS;  // 2 center offsets, 2 size values, then NUM_KEYPOINTS (x, y) keypoint offsets

constexpr std::size_t CONF_VALUES_PER_ANCHOR = 2;  // Non-face, then face class score

constexpr std::size_t IOU_VALUES_PER_ANCHOR = 1;

struct YuNetDetections {                       // Coordinates normalized to [0, 1] of the model input size; descending score order after NMS
    std::vector<std::array<float, 4>> bboxes;  // (xCenter, yCenter, width, height), index-aligned with scores
    std::vector<float> scores;
    std::vector<std::vector<std::array<float, 2>>> keypoints;
};

std::vector<std::array<float, 4>> generateAnchors(std::uint32_t inputWidth, std::uint32_t inputHeight);  // From github.com/Kazuhito00/YuNet-ONNX-TFLite-Sample

YuNetDetections computeYuNetDetections(const std::vector<float>& loc,  // Mirrors the source compute_yunet_detections; maxDetections <= 0: no limit
                                       const std::vector<float>& conf,
                                       const std::vector<float>& iou,
                                       const std::vector<std::array<float, 4>>& anchors,
                                       std::uint32_t inputWidth,
                                       std::uint32_t inputHeight,
                                       float confThreshold,
                                       float iouThreshold,
                                       int maxDetections);

}  // namespace YuNetUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
