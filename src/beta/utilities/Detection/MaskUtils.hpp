#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "depthai/utility/span.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace MaskUtils {

/**
 * @brief Sigmoid activation, mirroring the source parser utility sigmoid().
 *
 * Uses the numerically stable split formulation in single precision: 1 / (1 + exp(-x)) for
 * x >= 0 and exp(x) / (1 + exp(x)) otherwise. NaN input yields NaN.
 *
 * @param value Input value.
 * @return Sigmoid of the input value.
 */
float sigmoid(float value);

/**
 * @brief Zero a (height, width) mask outside a bounding box, mirroring the source crop_mask().
 *
 * The box is given as (xCenter, yCenter, width, height) in mask-pixel units. A pixel at
 * column r and row c is kept when r >= x1, r < x2, c >= y1 and c < y2, where
 * x1/x2 (y1/y2) are the box left/right (top/bottom) edges; other pixels are multiplied by
 * zero (a NaN pixel value stays NaN, matching the numpy multiplication).
 *
 * @param mask Row-major mask values of size maskHeight * maskWidth, cropped in place.
 * @param maskHeight Mask height.
 * @param maskWidth Mask width.
 * @param bboxCxcywh Bounding box as (xCenter, yCenter, width, height) in mask-pixel units.
 */
void cropMask(std::vector<float>& mask, std::size_t maskHeight, std::size_t maskWidth, const std::array<double, 4>& bboxCxcywh);

/**
 * @brief Nearest-neighbor resize of a (height, width) single-channel mask, replicating
 * cv2.resize(src, (dstWidth, dstHeight), interpolation=cv2.INTER_NEAREST).
 *
 * Follows OpenCV's resizeNN index mapping: a destination column dx samples the source column
 * min(floor(dx * (1.0 / (double(dstWidth) / srcWidth))), srcWidth - 1), and rows likewise.
 *
 * @param src Row-major source values of size srcHeight * srcWidth.
 * @param srcHeight Source height, must be greater than 0.
 * @param srcWidth Source width, must be greater than 0.
 * @param dstHeight Destination height.
 * @param dstWidth Destination width.
 * @return Row-major resized values of size dstHeight * dstWidth.
 */
std::vector<std::uint8_t> resizeNearest(
    const std::vector<std::uint8_t>& src, std::size_t srcHeight, std::size_t srcWidth, std::size_t dstHeight, std::size_t dstWidth);

/**
 * @brief Process a single RF-DETR instance segmentation mask, mirroring the source
 * process_single_mask_rfdetr().
 *
 * The bounding box, given as (xCenter, yCenter, width, height) normalized to [0, 1], is scaled
 * to the mask dimensions. The mask logits are passed through a sigmoid, cropped to the scaled
 * box, binarized with a strict greater-than maskConf threshold to 0/1 and resized to
 * (outputHeight, outputWidth) with nearest-neighbor interpolation.
 *
 * @param maskLogits Row-major mask logits of size maskHeight * maskWidth.
 * @param maskHeight Mask logits height, must be greater than 0.
 * @param maskWidth Mask logits width, must be greater than 0.
 * @param maskConf Mask confidence threshold, applied with a strict greater-than comparison.
 * @param bboxCxcywh Bounding box as (xCenter, yCenter, width, height), normalized to [0, 1].
 * @param outputHeight Output mask height.
 * @param outputWidth Output mask width.
 * @return Row-major binarized mask of size outputHeight * outputWidth with values 0 and 1.
 */
std::vector<std::uint8_t> processSingleMaskRfdetr(span<const float> maskLogits,
                                                  std::size_t maskHeight,
                                                  std::size_t maskWidth,
                                                  float maskConf,
                                                  const std::array<float, 4>& bboxCxcywh,
                                                  std::size_t outputHeight,
                                                  std::size_t outputWidth);

}  // namespace MaskUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
