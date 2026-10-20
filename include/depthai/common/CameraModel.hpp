#pragma once

#include <cstdint>
#include <string_view>

namespace dai {
/**
 * Projection model of a camera.
 *
 * `Perspective`, `Fisheye` and `RadialDivision` are pinhole cameras whose distortion is applied on the normalized image plane. `Equirectangular` and
 * `Cylindrical` are the surfaces panoramas are rendered onto; they carry no distortion coefficients, and their intrinsic matrix (fx, fy, cx, cy) maps the
 * angular coordinates of a direction (x, y, z) in the camera frame to a pixel instead of the normalized image plane:
 *  - Equirectangular: u = fx * atan2(x, z) + cx, v = fy * asin(y / sqrt(x^2 + y^2 + z^2)) + cy
 *  - Cylindrical: u = fx * atan2(x, z) + cx, v = fy * y / sqrt(x^2 + z^2) + cy
 *
 * so fx and fy are the radius of the sphere or cylinder in pixels, and (cx, cy) is the pixel the camera Z axis projects to.
 */
enum class CameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3, Cylindrical = 4 };

[[nodiscard]] constexpr std::string_view toString(CameraModel model) {
    switch(model) {
        case CameraModel::Perspective:
            return "Perspective";
        case CameraModel::Fisheye:
            return "Fisheye";
        case CameraModel::Equirectangular:
            return "Equirectangular";
        case CameraModel::RadialDivision:
            return "RadialDivision";
        case CameraModel::Cylindrical:
            return "Cylindrical";
    }

    return "Unknown";
}

}  // namespace dai
