#pragma once

#include <cstdint>
#include <string_view>

namespace dai {
/**
 * Which CameraModel to initialize the calibration with.
 */
enum class CameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 };

[[nodiscard]] constexpr std::string_view toString(CameraModel model) {
    switch(model) {
        case CameraModel::Perspective:    return "Perspective";
        case CameraModel::Fisheye:         return "Fisheye";
        case CameraModel::Equirectangular: return "Equirectangular";
        case CameraModel::RadialDivision:  return "RadialDivision";
    }

    return "Unknown";
}

}  // namespace dai
