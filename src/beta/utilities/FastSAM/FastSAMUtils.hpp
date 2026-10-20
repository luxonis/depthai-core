// This file contains utility functions for decoding the output of the FastSAM instance
// segmentation model, ported from the depthai-nodes parser utilities (utils/fastsam.py,
// utils/yolo.py, utils/nms.py and utils/masks_utils.py).
//
// Several functions are based on code from the Ultralytics FastSAM implementation; the
// original per-function source attributions of the depthai-nodes utilities are retained
// verbatim in the function documentation below.

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
namespace FastSAMUtils {

/**
 * @brief A decoded detection row after non-maximum suppression, mirroring the source
 * decode_fastsam_output() output row [x1, y1, x2, y2, conf, cls, head, anchor, x, y].
 *
 * The box corners are in model-input pixels, conf is the class confidence, cls the class
 * index, and (head, anchor, x, y) address the grid cell of the detection's YOLO head for the
 * mask-coefficient lookup. All values are stored in double precision, matching the float64
 * arrays the source pipeline carries after the per-head parse.
 */
using DetectionRow = std::array<double, 10>;

/**
 * @brief Prompt configuration of the FastSAM decode, mirroring the source parser attributes.
 */
struct PromptConfig {
    /// Prompt type: "everything", "bbox" or "point".
    std::string prompt = "everything";
    /// Prompt point as (x, y) in model-input pixels; required for the "point" prompt.
    std::optional<std::pair<std::int32_t, std::int32_t>> points;
    /// Prompt point label: 1 adds the masks containing the point, 0 subtracts them; required
    /// for the "point" prompt.
    std::optional<std::int32_t> pointLabel;
    /// Prompt bounding box as (x1, y1, x2, y2) in model-input pixels; required for the "bbox"
    /// prompt.
    std::optional<std::array<std::int32_t, 4>> bbox;
};

/**
 * @brief Merged FastSAM segmentation mask and the number of instance masks it encodes.
 */
struct FastsamResult {
    /// Row-major (height, width) mask; a pixel holds the index of the instance it belongs to,
    /// 255 marks background.
    std::vector<std::uint8_t> mergedMask;
    /// Mask width (model input width).
    std::size_t width = 0;
    /// Mask height (model input height).
    std::size_t height = 0;
    /// Number of instance masks merged into the mask; 0 when nothing was detected.
    std::size_t maskCount = 0;
};

/**
 * @brief Greedy non-maximum suppression, mirroring the source utils/nms.py nms().
 *
 * Boxes are given as corner-form rows (x1, y1, x2, y2) with a score per box; all arithmetic is
 * performed in single precision like the source, which casts the stacked array to float32. Box
 * areas use the +1 convention (x2 - x1 + 1) * (y2 - y1 + 1). Candidates are visited in
 * descending score order (equal scores keep the reversed ascending-argsort order, i.e. the
 * later index first) and a candidate suppresses the remaining boxes whose overlap is strictly
 * greater than the threshold.
 *
 * @param boxes Corner-form boxes (x1, y1, x2, y2), index-aligned with scores.
 * @param scores Confidence scores, one per box.
 * @param nmsThreshold Overlap threshold; remaining boxes with overlap <= threshold survive.
 * @return Indices of the kept boxes in descending score order.
 */
std::vector<std::size_t> nms(const std::vector<std::array<float, 4>>& boxes, const std::vector<float>& scores, float nmsThreshold);

/**
 * @brief Adjust bounding boxes to stick to the image border when they are within a threshold,
 * mirroring the source adjust_bboxes_to_image_border().
 *
 * Source: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/utils.py#L6 (Ultralytics)
 *
 * Boxes are corner-form detection rows modified in place: x1 < threshold snaps to 0,
 * y1 < threshold snaps to 0, x2 > width - threshold snaps to width and y2 > height - threshold
 * snaps to height.
 *
 * @param rows Detection rows whose first four columns are the corner-form box, adjusted in
 *             place.
 * @param imgHeight Image height.
 * @param imgWidth Image width.
 * @param threshold Pixel threshold. Defaults to 20 like the source.
 */
void adjustBboxesToImageBorder(std::vector<DetectionRow>& rows, std::int64_t imgHeight, std::int64_t imgWidth, std::int64_t threshold = 20);

/**
 * @brief Compute the IoU of one box against every detection row's box and return the indices
 * with IoU strictly greater than the threshold, mirroring the source bbox_iou().
 *
 * Source: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/utils.py#L30 (Ultralytics - rewritten to numpy)
 *
 * Like the source, the rows are first border-snapped in place with
 * adjustBboxesToImageBorder(); this side effect is part of the decode contract. The
 * intersection uses max(0, x2 - x1) * max(0, y2 - y1) without the +1 convention.
 *
 * @param box1 Single corner-form box (x1, y1, x2, y2).
 * @param rows Detection rows whose boxes are compared (and border-snapped in place).
 * @param iouThres IoU threshold, applied with a strict greater-than comparison. Defaults to
 *                 0.9 like the source.
 * @param imgHeight Image height used for the border snap.
 * @param imgWidth Image width used for the border snap.
 * @return Indices of rows with IoU > iouThres.
 */
std::vector<std::size_t> bboxIou(
    const std::array<double, 4>& box1, std::vector<DetectionRow>& rows, std::int64_t imgHeight, std::int64_t imgWidth, double iouThres = 0.9);

/**
 * @brief Decode the FastSAM YOLO outputs into NMS-filtered detection rows, mirroring the
 * source decode_fastsam_output() (which drives decode_yolo_output()/parse_yolo_output() in
 * anchorless segmentation mode with strides [8, 16, 32]) including the full-box augmentation.
 *
 * Each YOLO output must be an NCHW tensor of shape (1, numClasses + 5, gridH, gridW) whose
 * channels are (x, y, w, h, objectness, class scores...). The anchorless box decode computes
 * x1y1 = grid - out[0:2] + 0.5 and x2y2 = grid + out[2:4] + 0.5, converts to center form and
 * scales by the head's stride, rounding through float32 like the source in-place numpy store.
 * Candidates pass an early objectness filter (strictly greater than confThres), are
 * concatenated across heads and, when more than 3000 candidates with non-identical objectness
 * remain, sorted by descending objectness and truncated to 3000. Non-maximum suppression then
 * converts boxes to corner form, keeps the best class per candidate (argmax over the class
 * scores), refilters with the class confidence, truncates to the 3000 first candidates of an
 * ascending confidence sort when exceeded (a source quirk kept for parity), offsets boxes by
 * class * 7680, runs nms() and keeps at most 300 detections (the source non_max_suppression
 * max_det default; the parser never overrides it).
 *
 * Finally the full-box augmentation border-snaps all boxes in place through bboxIou() and,
 * when any box has IoU > 0.9 with the full-image box, replaces the highest-confidence such row
 * with the full-image box (keeping that row's confidence and grid-cell columns and setting its
 * class to 0).
 *
 * @param yoloOutputs YOLO output tensors in NCHW orientation, sorted by layer name; exactly
 *                    as many outputs as strides (3) are required.
 * @param imgHeight Model input height (gridH of the first output * 8).
 * @param imgWidth Model input width (gridW of the first output * 8).
 * @param confThres Confidence threshold, applied with strict greater-than comparisons.
 * @param iouThres NMS overlap threshold.
 * @param numClasses Number of classes in the model.
 * @return NMS-filtered detection rows, empty when nothing passes the filters.
 */
std::vector<DetectionRow> decodeFastsamOutput(const std::vector<ClassificationUtils::ShapedTensorData>& yoloOutputs,
                                              std::int64_t imgHeight,
                                              std::int64_t imgWidth,
                                              float confThres,
                                              float iouThres,
                                              std::int32_t numClasses);

/**
 * @brief Gather the mask coefficients for all detections, grouped by head, mirroring the
 * source build_mask_coeffs().
 *
 * For each detection the (head, anchor, x, y) columns select the mask output of its head and
 * the protosLen coefficients at channels anchor * protosLen + [0, protosLen) of grid cell
 * (y, x). Rows whose head index matches no mask output keep zero coefficients (the source
 * leaves them uninitialized).
 *
 * @param rows Decoded detection rows.
 * @param maskOutputs Mask output tensors in NCHW orientation, sorted by layer name and
 *                    index-aligned with the sorted YOLO outputs.
 * @param protosLen Number of prototypes (channel count of the protos tensor).
 * @return Row-major mask coefficients of shape (rows.size(), protosLen).
 */
std::vector<float> buildMaskCoeffs(const std::vector<DetectionRow>& rows,
                                   const std::vector<ClassificationUtils::ShapedTensorData>& maskOutputs,
                                   std::size_t protosLen);

/**
 * @brief Process the decoded detections into full-size binary masks, mirroring the source
 * process_masks().
 *
 * Per detection, the prototype masks are combined with the detection's coefficients
 * (sequential float32 accumulation like numpy's axis-0 sum), passed through a sigmoid, resized
 * to (outHeight, outWidth) with nearest-neighbor interpolation, zeroed outside the detection's
 * box (truncated to int and clamped to the output size) and binarized with a strict
 * greater-than maskConf threshold.
 *
 * @param rows Decoded detection rows.
 * @param maskCoeffs Row-major mask coefficients of shape (rows.size(), protosLen).
 * @param protos Protos tensor values of shape (protosLen, protoH, protoW) (the leading batch
 *               dimension already indexed away).
 * @param protosLen Number of prototypes.
 * @param protoHeight Prototype mask height.
 * @param protoWidth Prototype mask width.
 * @param outHeight Output mask height (model input height).
 * @param outWidth Output mask width (model input width).
 * @param maskConf Mask confidence threshold, applied with a strict greater-than comparison.
 * @return Row-major binary masks of shape (rows.size(), outHeight, outWidth) with values 0/1.
 */
std::vector<std::uint8_t> processMasks(const std::vector<DetectionRow>& rows,
                                       const std::vector<float>& maskCoeffs,
                                       const std::vector<float>& protos,
                                       std::size_t protosLen,
                                       std::size_t protoHeight,
                                       std::size_t protoWidth,
                                       std::size_t outHeight,
                                       std::size_t outWidth,
                                       float maskConf);

/**
 * @brief Select the mask with the highest IoU against the prompt bounding box, mirroring the
 * source box_prompt().
 *
 * Source: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/prompt.py#L286
 * Modified so it uses numpy instead of torch.
 *
 * The box is scaled when the mask shape differs from the original shape (never in this parser,
 * where both are the model input size), clamped to the mask bounds, and the IoU of each mask's
 * pixels inside the box against the union of the box and the mask is computed. The mask with
 * the maximum IoU is returned (first index on ties; a NaN IoU from an empty union wins like
 * numpy's argmax).
 *
 * @param masks Row-major binary masks of shape (maskCount, height, width).
 * @param maskCount Number of masks; must be greater than 0.
 * @param height Mask height.
 * @param width Mask width.
 * @param bbox Prompt bounding box as (x1, y1, x2, y2) in model-input pixels; x2 and y2 must
 *             not be 0 (source assertion).
 * @return The selected mask of shape (1, height, width).
 */
std::vector<std::uint8_t> boxPrompt(
    const std::vector<std::uint8_t>& masks, std::size_t maskCount, std::size_t height, std::size_t width, const std::array<std::int32_t, 4>& bbox);

/**
 * @brief Combine the masks containing the prompt point into a single mask, mirroring the
 * source point_prompt() (with format_results() folded in; the annotation id/bbox/score/area
 * fields it builds do not influence the result).
 *
 * format_results() source: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/prompt.py#L56
 * point_prompt() source: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/prompt.py#L321
 * Modified so it uses numpy instead of torch.
 *
 * Every mask whose pixel at the point equals 1 is added to an accumulator when the point label
 * is 1 and subtracted when the label is 0 (other labels ignore the mask). The result is the
 * accumulator thresholded at >= 1. Negative point coordinates wrap around like numpy indexing;
 * out-of-range coordinates raise an error like the source IndexError.
 *
 * @param masks Row-major binary masks of shape (maskCount, height, width).
 * @param maskCount Number of masks; must be greater than 0.
 * @param height Mask height.
 * @param width Mask width.
 * @param point Prompt point as (x, y) in model-input pixels.
 * @param pointLabel Prompt point label; 1 adds, 0 subtracts.
 * @return The combined mask of shape (1, height, width).
 */
std::vector<std::uint8_t> pointPrompt(const std::vector<std::uint8_t>& masks,
                                      std::size_t maskCount,
                                      std::size_t height,
                                      std::size_t width,
                                      const std::pair<std::int32_t, std::int32_t>& point,
                                      std::int32_t pointLabel);

/**
 * @brief Merge instance masks into a single (height, width) mask, mirroring the source
 * merge_masks().
 *
 * The merged mask starts as background (255) and every mask's non-zero pixels are assigned the
 * mask's index in ascending order, so later (lower-confidence) instances overwrite earlier
 * ones on overlap. More than 256 masks cannot be encoded (index 256 would not fit uint8; the
 * source numpy assignment raises an OverflowError).
 *
 * @param masks Row-major binary masks of shape (maskCount, height, width).
 * @param maskCount Number of masks.
 * @param height Mask height.
 * @param width Mask width.
 * @return The merged mask of shape (height, width).
 */
std::vector<std::uint8_t> mergeMasks(const std::vector<std::uint8_t>& masks, std::size_t maskCount, std::size_t height, std::size_t width);

/**
 * @brief Decode the FastSAM outputs into a merged segmentation mask and mask count, mirroring
 * the source compute_fastsam_mask().
 *
 * The model input size is derived from the first (sorted) YOLO output as gridW * 8 and
 * gridH * 8 (the stride-8 head). The YOLO outputs are decoded with decodeFastsamOutput(); with
 * no detections the result is a fully-background mask with count 0 (the source merges a
 * sentinel int16 array of -1, which produces the same all-255 bytes). Otherwise the detections
 * are turned into binary masks through buildMaskCoeffs() and processMasks(), reduced by the
 * "bbox" or "point" prompt when configured, and merged with mergeMasks().
 *
 * @param yoloOutputs YOLO output tensors in NCHW orientation, sorted by layer name.
 * @param maskOutputs Mask output tensors in NCHW orientation, sorted by layer name.
 * @param protos Protos tensor in NCHW orientation of shape (1, protosLen, protoH, protoW).
 * @param protosLen Number of prototypes (protos channel count).
 * @param confThreshold Confidence threshold.
 * @param numClasses Number of classes in the model.
 * @param iouThreshold NMS overlap threshold.
 * @param maskConf Mask confidence threshold.
 * @param promptConfig Prompt configuration.
 * @return Merged segmentation mask with its dimensions and the mask count.
 */
FastsamResult computeFastsamMask(const std::vector<ClassificationUtils::ShapedTensorData>& yoloOutputs,
                                 const std::vector<ClassificationUtils::ShapedTensorData>& maskOutputs,
                                 const ClassificationUtils::ShapedTensorData& protos,
                                 std::size_t protosLen,
                                 float confThreshold,
                                 std::int32_t numClasses,
                                 float iouThreshold,
                                 float maskConf,
                                 const PromptConfig& promptConfig);

}  // namespace FastSAMUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
