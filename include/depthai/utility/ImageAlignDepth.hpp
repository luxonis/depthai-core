#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace dai {
namespace impl {

// A remap changes pixel rays, but RAW16 still contains the old camera's axial Z.
// Cache 1 / (R_destination_to_source * K_destination^-1 * [u,v,1]).z.
// The disparity shift uses the non-inverted source/rectified Z ratio instead.
inline std::vector<float> imageAlignDepthScales(int width, int height, const std::array<float, 3>& sourceZFromDestinationPixel, bool invert = true) {
    std::vector<float> scales(static_cast<size_t>(width) * height);
    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
            const float z = sourceZFromDestinationPixel[0] * x + sourceZFromDestinationPixel[1] * y + sourceZFromDestinationPixel[2];
            scales[static_cast<size_t>(y) * width + x] = std::isfinite(z) && z > 1e-6f ? (invert ? 1.0f / z : z) : 0.0f;
        }
    }
    return scales;
}

inline uint16_t imageAlignScaleDepth(uint16_t depth, float scale, float offset = 0.0f) {
    if(depth == 0) return 0;
    // Z_target = (Z_source + (R^T * t).z) / (R^T * ray_target).z.
    const float z = (depth + offset) * scale;
    // Preserve invalid depth and discard points behind the camera or outside RAW16.
    return z >= 0.5f && z < 65535.5f ? static_cast<uint16_t>(z + 0.5f) : 0;
}

inline void imageAlignConvertDepth(uint8_t* data, int width, int height, int stride, const float* scales, float offset = 0.0f) {
    for(int y = 0; y < height; ++y) {
        auto* row = reinterpret_cast<uint16_t*>(data + static_cast<size_t>(y) * stride);
        for(int x = 0; x < width; ++x) {
            row[x] = imageAlignScaleDepth(row[x], scales[static_cast<size_t>(y) * width + x], offset);
        }
    }
}

}  // namespace impl
}  // namespace dai
