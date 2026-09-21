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

/**
 * @brief Options for the SSD anchor generator, mirroring MediaPipe's SSDAnchorOptions.
 *
 * See
 * https://github.com/google/mediapipe/blob/master/mediapipe/calculators/tflite/ssd_anchors_calculator.cc
 */
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

/**
 * @brief Generate SSD anchors for the given options, mirroring MediaPipe's
 * ssd_anchors_calculator.
 *
 * @param options SSD anchor generator options.
 * @return Anchors as [xCenter, yCenter, width, height], normalized by the input size.
 */
std::vector<std::array<double, 4>> generateAnchors(const SSDAnchorOptions& options);

/**
 * @brief Generate the SSD anchors of the MediaPipe palm (hand) detection model for the given
 * input size.
 *
 * Uses the anchor options of MediaPipe's palm_detection_cpu.pbtxt: 4 layers with strides
 * [8, 16, 16, 16], scales in [0.1484375, 0.75], aspect ratio 1.0, interpolated scale aspect
 * ratio 1.0 and fixed anchor size.
 *
 * @param inputSizeWidth Model input width in pixels.
 * @param inputSizeHeight Model input height in pixels.
 * @return Anchors as [xCenter, yCenter, width, height], normalized by the input size.
 */
std::vector<std::array<double, 4>> generateHandtrackerAnchors(int inputSizeWidth, int inputSizeHeight);

/**
 * @brief Decoded MediaPipe palm detections after non-maximum suppression, ordered by descending
 * confidence score.
 */
struct PalmDetections {
    /// Bounding boxes as [xCenter, yCenter, width, height], normalized to [0, 1].
    std::vector<std::array<float, 4>> bboxes;
    /// Confidence scores, index-aligned with bboxes.
    std::vector<float> scores;
    /// Rotation angles of the bounding boxes in degrees, rounded to 0 decimals (half to even),
    /// index-aligned with bboxes.
    std::vector<float> angles;
};

/**
 * @brief Decode the raw MediaPipe palm detection tensors into rotated bounding boxes and apply
 * non-maximum suppression, mirroring the source compute_mediapipe_palm_detections utility.
 *
 * Scores are passed through a sigmoid and filtered with a strict greater-than confidence
 * threshold. The kept rows are decoded against the SSD anchors: each of the 18 values (bounding
 * box center/size plus 7 keypoint coordinate pairs) is scaled by the anchor size over the input
 * scale and offset by the anchor center; boxes with a negative decoded width or height are
 * skipped. Each detection is converted to a rectangle rotated so that the line from the wrist
 * keypoint to the middle-finger keypoint aligns with the rectangle's y-axis, expanded to a square
 * over the long side and converted to its 4 integer corner points in pixels. The rectangle's
 * angle, center and size are derived from the corner points and NMS is applied to the axis-aligned
 * [xMin, yMin, width, height] boxes with the confidence threshold, the IoU threshold and maxDet as
 * the top-k limit. The kept boxes are normalized by the scale and clipped to [0, 1]; the kept
 * angles are rounded to 0 decimals.
 *
 * @param bboxes Raw bounding box tensor values of shape (numAnchors, 18), stored row-major.
 * @param scores Raw score tensor values of shape (numAnchors,).
 * @param anchors SSD anchors as [xCenter, yCenter, width, height]; the anchor count must match
 *                the score count when any score passes the confidence threshold.
 * @param confThreshold Confidence score threshold, applied to the sigmoid scores with a strict
 *                      greater-than.
 * @param iouThreshold Non-maximum suppression IoU threshold.
 * @param maxDet Maximum number of detections to keep (NMS top-k limit; no limit when 0 or
 *               negative).
 * @param scale Scale (input size) of the model in pixels, used to decode and normalize the boxes.
 * @return Palm detections after non-maximum suppression.
 */
PalmDetections computeMediaPipePalmDetections(const std::vector<float>& bboxes,
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
