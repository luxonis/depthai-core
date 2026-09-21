#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace RFDETRUtils {

/**
 * @brief Decoded RF-DETR detections and the optional instance segmentation mask.
 */
struct RfDetrDetections {
    /// Bounding boxes as (xCenter, yCenter, width, height), normalized to [0, 1], ordered by
    /// descending score.
    std::vector<std::array<float, 4>> bboxes;
    /// Confidence scores, index-aligned with bboxes.
    std::vector<float> scores;
    /// Class labels, index-aligned with bboxes.
    std::vector<std::uint32_t> labels;
    /// Label names, index-aligned with bboxes; empty when no label names are configured.
    std::vector<std::string> labelNames;
    /// Row-major instance segmentation mask of shape (maskHeight, maskWidth); a pixel holds the
    /// index of the detection it belongs to, 255 marks background. Empty in detection mode.
    std::vector<std::uint8_t> segmentationMask;
    /// Segmentation mask width; 0 in detection mode.
    std::size_t maskWidth = 0;
    /// Segmentation mask height; 0 in detection mode.
    std::size_t maskHeight = 0;
    /// Number of confident instances beyond the 255 the segmentation mask can encode; greater
    /// than 0 signals the caller to log the source parser's warning. Always 0 in detection mode.
    std::size_t ignoredInstances = 0;
};

/**
 * @brief Decode RF-DETR detections and the optional instance segmentation mask, mirroring the
 * source compute_rfdetr_detections().
 *
 * The class probabilities are the sigmoid of the logits; per query, the score is the maximum
 * probability over the classes and the label its first index (numpy max/argmax semantics,
 * including NaN propagation). Queries are ordered by descending score with numpy's
 * argsort()[::-1] tie ordering and truncated to maxDetections; in segmentation mode the
 * truncation is additionally capped at 255, the maximum number of instances the segmentation
 * mask can encode, and ignoredInstances reports how many confident instances the cap dropped.
 * The (xCenter, yCenter, width, height) boxes are converted to corner form, clipped to [0, 1]
 * and filtered with a strict greater-than confidence threshold together with the scores,
 * labels and mask logits.
 *
 * In segmentation mode the final mask of the model input shape starts as background (255) and
 * every detection's processed mask (see MaskUtils::processSingleMaskRfdetr(), driven by the
 * unclipped center-form box) claims its still-background pixels with the detection index, in
 * descending score order. The filtered corner-form boxes are converted back to center form,
 * validating xMin <= xMax and yMin <= yMax like the source xyxy_to_xywh(). When label names
 * are configured, each detection maps its label to the matching name, or to "class_<label>"
 * when the label is out of range.
 *
 * @param boxesTensor Boxes tensor with shape squeezing to (N, 4), boxes as normalized
 *                    (xCenter, yCenter, width, height).
 * @param logitsTensor Class logits tensor with shape (1, N, C).
 * @param masksTensor Optional mask logits tensor with shape squeezing to (N, maskH, maskW);
 *                    its presence selects segmentation mode.
 * @param confThreshold Confidence score threshold, applied with a strict greater-than
 *                      comparison.
 * @param maxDetections Maximum number of detections to keep, must be greater than 0.
 * @param labelNames Label names indexed by class label; empty when not configured.
 * @param maskConf Mask confidence threshold for binarizing instance masks.
 * @param inputSize Model input size as (width, height); required in segmentation mode, where
 *                  it defines the final mask shape.
 * @return Decoded detections with the optional instance segmentation mask.
 */
RfDetrDetections computeRfDetrDetections(const ClassificationUtils::ShapedTensorData& boxesTensor,
                                         const ClassificationUtils::ShapedTensorData& logitsTensor,
                                         const std::optional<ClassificationUtils::ShapedTensorData>& masksTensor,
                                         float confThreshold,
                                         int maxDetections,
                                         const std::vector<std::string>& labelNames,
                                         float maskConf,
                                         const std::optional<std::pair<std::uint32_t, std::uint32_t>>& inputSize);

}  // namespace RFDETRUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
