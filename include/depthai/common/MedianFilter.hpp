#pragma once

// std
#include <cstdint>
namespace dai {

/**
 * Median filter config
 */
enum class MedianFilter : int32_t { MEDIAN_OFF = 0, KERNEL_3x3 = 3, KERNEL_5x5 = 5, KERNEL_7x7 = 7 };

}  // namespace dai