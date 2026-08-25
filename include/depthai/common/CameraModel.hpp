#pragma once

#include <cstdint>
#include <string>

namespace dai {
/**
 * Which CameraModel to initialize the calibration with.
 */
enum class CameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 };

inline std::string toString(CameraModel model) {
    switch(model) {
        case CameraModel::Perspective:
            return "Perspective";
        case CameraModel::Fisheye:
            return "Fisheye";
        case CameraModel::RadialDivision:
            return "RadialDivision";
        case CameraModel::Equirectangular:
            return "Equirectangular";
    }
    return "Unknown";
}

}  // namespace dai
