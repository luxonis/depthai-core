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

/// Number of keypoints per YuNet detection. YuNet models predict 5 facial keypoints (10
/// coordinate values) per anchor; this is part of the YuNet output contract, not a per-model
/// value.
constexpr std::size_t NUM_KEYPOINTS = 5;

/// Number of loc values per YuNet anchor: 2 bounding box center offsets, 2 bounding box size
/// values and NUM_KEYPOINTS (x, y) keypoint offset pairs.
constexpr std::size_t LOC_VALUES_PER_ANCHOR = 4 + 2 * NUM_KEYPOINTS;

/// Number of conf values per YuNet anchor (non-face and face class scores).
constexpr std::size_t CONF_VALUES_PER_ANCHOR = 2;

/// Number of iou values per YuNet anchor.
constexpr std::size_t IOU_VALUES_PER_ANCHOR = 1;

/**
 * @brief Decoded YuNet detections.
 *
 * All coordinates are normalized to [0, 1] against the model input size and clipped.
 * Detections are ordered by descending score after non-maximum suppression.
 */
struct YuNetDetections {
    /// Bounding boxes as (xCenter, yCenter, width, height), index-aligned with scores.
    std::vector<std::array<float, 4>> bboxes;
    /// Confidence scores of the detections.
    std::vector<float> scores;
    /// Keypoints per detection as NUM_KEYPOINTS (x, y) pairs, index-aligned with scores.
    std::vector<std::vector<std::array<float, 2>>> keypoints;
};

/**
 * @brief Generate a set of default bounding boxes, known as anchors, mirroring the source
 * generate_anchors. The code is taken from https://github.com/Kazuhito00/YuNet-ONNX-TFLite-Sample/tree/main
 *
 * The sizes of the feature maps are calculated by progressively halving the dimensions of the
 * input image with truncating integer division: the second level is
 * (int(int((h + 1) / 2) / 2), int(int((w + 1) / 2) / 2)) and every following level halves the
 * previous one; the third to sixth levels carry anchors. Level k uses the minimum sizes
 * [[10, 16, 24], [32, 48], [64, 96], [128, 192, 256]][k] and the stride [8, 16, 32, 64][k]
 * (the source defaults; the source parser always uses them). Per feature map position
 * (row i, column j, row-major) and minimum size, the anchor is
 * (cx, cy, sKx, sKy) = ((j + 0.5) * stride / w, (i + 0.5) * stride / h, minSize / w,
 * minSize / h), computed in double precision and stored in single precision like the source's
 * float32 anchor array.
 *
 * @param inputWidth Model input image width, must be greater than 0.
 * @param inputHeight Model input image height, must be greater than 0.
 * @return Anchors as (cx, cy, sKx, sKy), normalized by the input size.
 */
std::vector<std::array<float, 4>> generateAnchors(std::uint32_t inputWidth, std::uint32_t inputHeight);

/**
 * @brief Decode YuNet outputs into final detections, mirroring the source
 * compute_yunet_detections (decode_and_prune_detections, format_detections, nms_cv2,
 * top_left_wh_to_xywh and the final clipping). The box and keypoint decoding is taken from
 * https://github.com/Kazuhito00/YuNet-ONNX-TFLite-Sample/tree/main
 *
 * The candidate scores are sqrt(conf[:, 1] * clip(iou[:, 0], 0, 1)); candidates with a score
 * strictly greater than confThreshold are decoded against their anchors with the variances
 * [0.1, 0.2]: the box center is (anchorXy + loc[:, 0:2] * 0.1 * anchorWh) * (w, h), the box
 * size is (anchorWh * exp(loc[:, 2:4] * 0.2)) * (w, h) and keypoint i is
 * (anchorXy + loc[:, 4 + 2i : 6 + 2i] * 0.1 * anchorWh) * (w, h), all in single precision.
 * The boxes are converted to top-left (x, y, width, height) and normalized by the input size;
 * the keypoint coordinates are truncated to int32 before normalization, mirroring the source
 * format_detections' astype(np.int32). Non-maximum suppression with cv2.dnn.NMSBoxes semantics
 * (DetectionUtils::nmsBoxes) is applied to the normalized top-left boxes with confThreshold,
 * iouThreshold and maxDetections as the top-k limit. The kept boxes are converted to center
 * (x, y, width, height) and clipped to [0, 1]; the kept keypoints are clipped to [0, 1].
 *
 * Validates that the loc, conf and iou value counts describe the same number of candidates and
 * that the anchor count matches the candidate count (the source numpy mask requirement).
 *
 * @param loc Raw location tensor values, LOC_VALUES_PER_ANCHOR per anchor, row-major.
 * @param conf Raw confidence tensor values, CONF_VALUES_PER_ANCHOR per anchor, row-major.
 * @param iou Raw IoU tensor values, IOU_VALUES_PER_ANCHOR per anchor.
 * @param anchors Anchors as (cx, cy, sKx, sKy), one per candidate.
 * @param inputWidth Model input image width the x coordinates are scaled and normalized by.
 * @param inputHeight Model input image height the y coordinates are scaled and normalized by.
 * @param confThreshold Confidence score threshold (strictly greater than to keep).
 * @param iouThreshold Non-maximum suppression (IoU) threshold.
 * @param maxDetections Maximum number of detections (NMS top-k limit; no limit when 0 or
 *                      negative).
 * @return Decoded detections ordered by descending score.
 */
YuNetDetections computeYuNetDetections(const std::vector<float>& loc,
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
