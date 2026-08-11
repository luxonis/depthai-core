// This file contains utility functions for decoding the output of the PaddlePaddle OCR text
// detection model, ported from the depthai-nodes parser utilities (utils/ppdet.py and
// utils/bbox_format_converters.py).

#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace dai {
namespace beta {
namespace utilities {
namespace PPTextUtils {

/**
 * @brief Decoded PaddlePaddle text detections, mirroring the source
 * parse_paddle_detection_outputs() return values.
 */
struct PPTextDetections {
    /// Rotated bounding boxes as [xCenter, yCenter, width, height], normalized to the model
    /// input and clipped to [0, 1], index-aligned with scores.
    std::vector<std::array<float, 4>> bboxes;
    /// Bounding box angles in degrees, rounded to 0 decimals (round half to even),
    /// index-aligned with bboxes.
    std::vector<float> angles;
    /// Bounding box confidence scores, index-aligned with bboxes.
    std::vector<float> scores;
};

/**
 * @brief Parse the output of a PaddlePaddle Text Detection model from a mask of text
 * probabilities into rotated bounding boxes, mirroring the source
 * parse_paddle_detection_outputs().
 *
 * The predictions tensor must be of shape (1, 1, H, W) or (1, H, W, 1); the probability map is
 * thresholded with the mask threshold (exclusive) into a binary mask, dilated with a 3x3 ones
 * kernel, and its contours are extracted (RETR_LIST, CHAIN_APPROX_SIMPLE). Contours with an
 * area of at most 8 are dropped; when more than maxDetections contours remain, the
 * maxDetections largest by area are selected and processed in the order produced by numpy's
 * generic argpartition (introselect) on the negated areas. Per contour, the minimum-area
 * rotated rectangle is computed and its corners are ordered top-left, top-right, bottom-right,
 * bottom-left; rectangles whose smaller side is below 8 pixels are dropped; the score is the
 * mean prediction value inside the corner polygon shrunk by 2 pixels and detections with a
 * score below the bbox threshold (exclusive) are dropped; the kept rectangle is expanded by
 * sqrt(2) in both dimensions, normalized by the map width/height and its angle is rounded to 0
 * decimals.
 *
 * @param predictions Dequantized FP32 prediction values in row-major order over dims.
 * @param dims Predictions tensor shape; must be (1, 1, H, W) or (1, H, W, 1).
 * @param maskThreshold Threshold for the binary text mask; exclusive.
 * @param bboxThreshold Threshold for the bounding box scores; exclusive.
 * @param maxDetections Maximum number of candidate bounding boxes.
 * @return Decoded text detections.
 * @note Requires OpenCV support; throws a runtime error when built without it.
 */
PPTextDetections parsePaddleDetectionOutputs(
    const std::vector<float>& predictions, const std::vector<std::size_t>& dims, float maskThreshold, float bboxThreshold, int maxDetections);

}  // namespace PPTextUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
