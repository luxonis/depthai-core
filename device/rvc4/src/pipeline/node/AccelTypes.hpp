#pragma once

namespace dai {
namespace gate {

/// Interpolation of the hardware downscaler (image pyramid and scaledown stages)
enum class ScaledownInterpolation { BILINEAR = 0, BICUBIC = 1 };

/// Search direction of the hardware stereo matcher
enum class StereoSearchDir { L2R = 0, R2L = 1 };

}  // namespace gate
}  // namespace dai
