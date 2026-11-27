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

float sigmoid(float value);

void cropMask(std::vector<float>& mask, std::size_t maskHeight, std::size_t maskWidth, const std::array<double, 4>& bboxCxcywh);  // bboxCxcywh in mask pixels

std::vector<std::uint8_t> resizeNearest(
    const std::vector<std::uint8_t>& src, std::size_t srcHeight, std::size_t srcWidth, std::size_t dstHeight, std::size_t dstWidth);

std::vector<float> resizeNearest(const std::vector<float>& src, std::size_t srcHeight, std::size_t srcWidth, std::size_t dstHeight, std::size_t dstWidth);

std::vector<std::uint8_t> processSingleMaskRfdetr(span<const float> maskLogits,  // bboxCxcywh normalized to [0, 1]; returns a 0/1 mask
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
