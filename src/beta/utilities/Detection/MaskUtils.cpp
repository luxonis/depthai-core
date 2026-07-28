#include "beta/utilities/Detection/MaskUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace MaskUtils {

float sigmoid(float value) {
    // Split formulation of the source sigmoid(): both branches evaluate in single precision,
    // matching numpy's float32 computation. NaN input takes the negative branch and yields NaN.
    if(value >= 0.0f) {
        return 1.0f / (1.0f + std::exp(-value));
    }
    const float expValue = std::exp(value);
    return expValue / (1.0f + expValue);
}

void cropMask(std::vector<float>& mask, std::size_t maskHeight, std::size_t maskWidth, const std::array<double, 4>& bboxCxcywh) {
    DAI_CHECK_V(mask.size() == maskHeight * maskWidth, "Mask size {} does not match the mask shape ({}, {}).", mask.size(), maskHeight, maskWidth);

    // The box edges stay in double precision, matching the numpy float64 comparisons of the
    // integer pixel indices against the scaled box in the source crop_mask().
    const double x1 = bboxCxcywh[0] - bboxCxcywh[2] / 2.0;
    const double y1 = bboxCxcywh[1] - bboxCxcywh[3] / 2.0;
    const double x2 = bboxCxcywh[0] + bboxCxcywh[2] / 2.0;
    const double y2 = bboxCxcywh[1] + bboxCxcywh[3] / 2.0;

    for(std::size_t row = 0; row < maskHeight; ++row) {
        const bool rowInside = static_cast<double>(row) >= y1 && static_cast<double>(row) < y2;
        for(std::size_t column = 0; column < maskWidth; ++column) {
            const bool inside = rowInside && static_cast<double>(column) >= x1 && static_cast<double>(column) < x2;
            // Multiply by 0/1 instead of assigning so a NaN value outside the box stays NaN,
            // matching the numpy multiplication by a boolean mask.
            mask[row * maskWidth + column] *= inside ? 1.0f : 0.0f;
        }
    }
}

std::vector<std::uint8_t> resizeNearest(
    const std::vector<std::uint8_t>& src, std::size_t srcHeight, std::size_t srcWidth, std::size_t dstHeight, std::size_t dstWidth) {
    DAI_CHECK(srcHeight > 0 && srcWidth > 0, "Nearest-neighbor resize requires a non-empty source mask.");
    DAI_CHECK_V(src.size() == srcHeight * srcWidth, "Mask size {} does not match the mask shape ({}, {}).", src.size(), srcHeight, srcWidth);

    // OpenCV resizeNN index mapping: the inverse scale is computed as 1.0 / (dst / src) in
    // double precision and destination indices map through floor with a clamp to the last
    // source index.
    const double inverseScaleX = 1.0 / (static_cast<double>(dstWidth) / static_cast<double>(srcWidth));
    const double inverseScaleY = 1.0 / (static_cast<double>(dstHeight) / static_cast<double>(srcHeight));

    std::vector<std::size_t> columnOffsets(dstWidth);
    for(std::size_t dstColumn = 0; dstColumn < dstWidth; ++dstColumn) {
        const auto srcColumn = static_cast<std::size_t>(std::floor(static_cast<double>(dstColumn) * inverseScaleX));
        columnOffsets[dstColumn] = std::min(srcColumn, srcWidth - 1);
    }

    std::vector<std::uint8_t> dst(dstHeight * dstWidth);
    for(std::size_t dstRow = 0; dstRow < dstHeight; ++dstRow) {
        const auto srcRow = std::min(static_cast<std::size_t>(std::floor(static_cast<double>(dstRow) * inverseScaleY)), srcHeight - 1);
        const std::uint8_t* srcRowData = src.data() + srcRow * srcWidth;
        std::uint8_t* dstRowData = dst.data() + dstRow * dstWidth;
        for(std::size_t dstColumn = 0; dstColumn < dstWidth; ++dstColumn) {
            dstRowData[dstColumn] = srcRowData[columnOffsets[dstColumn]];
        }
    }
    return dst;
}

std::vector<std::uint8_t> processSingleMaskRfdetr(span<const float> maskLogits,
                                                  std::size_t maskHeight,
                                                  std::size_t maskWidth,
                                                  float maskConf,
                                                  const std::array<float, 4>& bboxCxcywh,
                                                  std::size_t outputHeight,
                                                  std::size_t outputWidth) {
    DAI_CHECK_V(maskLogits.size() == maskHeight * maskWidth,
                "RF-DETR mask logits size {} does not match the mask shape ({}, {}).",
                maskLogits.size(),
                maskHeight,
                maskWidth);
    DAI_CHECK(maskHeight > 0 && maskWidth > 0, "RF-DETR mask logits should have a non-empty (H, W) shape.");

    // scaled_bbox = bbox * [mask_w, mask_h, mask_w, mask_h]: the float32 box multiplied by the
    // integer dimensions promotes to float64 in the source, so the scaling stays in double.
    const std::array<double, 4> scaledBbox = {static_cast<double>(bboxCxcywh[0]) * static_cast<double>(maskWidth),
                                              static_cast<double>(bboxCxcywh[1]) * static_cast<double>(maskHeight),
                                              static_cast<double>(bboxCxcywh[2]) * static_cast<double>(maskWidth),
                                              static_cast<double>(bboxCxcywh[3]) * static_cast<double>(maskHeight)};

    std::vector<float> mask(maskLogits.size());
    for(std::size_t i = 0; i < mask.size(); ++i) {
        mask[i] = sigmoid(maskLogits[i]);
    }
    cropMask(mask, maskHeight, maskWidth, scaledBbox);

    std::vector<std::uint8_t> binary(mask.size());
    for(std::size_t i = 0; i < mask.size(); ++i) {
        binary[i] = mask[i] > maskConf ? 1 : 0;
    }

    return resizeNearest(binary, maskHeight, maskWidth, outputHeight, outputWidth);
}

}  // namespace MaskUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
