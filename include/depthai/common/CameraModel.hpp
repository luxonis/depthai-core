#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace dai {

/**
 * Projection model of a camera: how a direction (x, y, z) in the camera frame maps to the (undistorted) image coordinates that the intrinsic matrix
 * (fx, fy, cx, cy) turns into a pixel.
 *
 *  - Pinhole: the normalized image plane, u = fx * x / z + cx, v = fy * y / z + cy. The only projection with a normalized image plane, so the only one a
 *    DistortionModel other than `NoDistortion` can be applied to.
 *  - Equirectangular: the surface of a sphere, u = fx * atan2(x, z) + cx, v = fy * asin(y / sqrt(x^2 + y^2 + z^2)) + cy
 *  - Cylindrical: the surface of a cylinder, u = fx * atan2(x, z) + cx, v = fy * y / sqrt(x^2 + z^2) + cy
 *
 * For the panorama surfaces fx and fy are the radius of the sphere or cylinder in pixels, and (cx, cy) is the pixel the camera Z axis projects to.
 */
enum class CameraProjectionModel : int8_t { Pinhole = 0, Equirectangular = 1, Cylindrical = 2 };

/**
 * Lens distortion model: how the distortion coefficients warp the normalized image plane of a Pinhole projection.
 *
 *  - NoDistortion: no distortion, the coefficients are ignored.
 *  - BrownConrady: the OpenCV standard model, [k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, taux, tauy] (OpenCV's `cv::projectPoints`).
 *    Calibrations call this the Perspective camera model.
 *  - KannalaBrandt: the OpenCV fisheye model, [k1, k2, k3, k4] (OpenCV's `cv::fisheye`). Calibrations call this the Fisheye camera model.
 *  - RadialDivision: the radial division model. Not supported by the (un)distortion helpers unless the coefficients are all zero.
 */
enum class DistortionModel : int8_t { NoDistortion = 0, BrownConrady = 1, KannalaBrandt = 2, RadialDivision = 3 };

/**
 * Camera model as stored in calibration data (CameraInfo::cameraType). It combines a CameraProjectionModel with a DistortionModel:
 *
 *  - Perspective: Pinhole projection with BrownConrady distortion
 *  - Fisheye: Pinhole projection with KannalaBrandt distortion
 *  - RadialDivision: Pinhole projection with RadialDivision distortion
 *  - Equirectangular: Equirectangular projection, no distortion
 *  - Cylindrical: Cylindrical projection, no distortion
 *
 * Use projectionModelOf() and distortionModelOf() to split it, and toCameraModel() to combine a pair back.
 */
enum class CameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3, Cylindrical = 4 };

[[nodiscard]] constexpr std::string_view toString(CameraProjectionModel model) {
    switch(model) {
        case CameraProjectionModel::Pinhole:
            return "Pinhole";
        case CameraProjectionModel::Equirectangular:
            return "Equirectangular";
        case CameraProjectionModel::Cylindrical:
            return "Cylindrical";
    }

    return "Unknown";
}

[[nodiscard]] constexpr std::string_view toString(DistortionModel model) {
    switch(model) {
        case DistortionModel::NoDistortion:
            return "NoDistortion";
        case DistortionModel::BrownConrady:
            return "BrownConrady";
        case DistortionModel::KannalaBrandt:
            return "KannalaBrandt";
        case DistortionModel::RadialDivision:
            return "RadialDivision";
    }

    return "Unknown";
}

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

/// Projection model of a combined CameraModel.
[[nodiscard]] constexpr CameraProjectionModel projectionModelOf(CameraModel model) {
    switch(model) {
        case CameraModel::Perspective:
        case CameraModel::Fisheye:
        case CameraModel::RadialDivision:
            return CameraProjectionModel::Pinhole;
        case CameraModel::Equirectangular:
            return CameraProjectionModel::Equirectangular;
        case CameraModel::Cylindrical:
            return CameraProjectionModel::Cylindrical;
    }

    return CameraProjectionModel::Pinhole;
}

/// Distortion model of a combined CameraModel.
[[nodiscard]] constexpr DistortionModel distortionModelOf(CameraModel model) {
    switch(model) {
        case CameraModel::Perspective:
            return DistortionModel::BrownConrady;
        case CameraModel::Fisheye:
            return DistortionModel::KannalaBrandt;
        case CameraModel::RadialDivision:
            return DistortionModel::RadialDivision;
        case CameraModel::Equirectangular:
        case CameraModel::Cylindrical:
            return DistortionModel::NoDistortion;
    }

    return DistortionModel::NoDistortion;
}

/**
 * Whether a projection model can carry the given distortion model. Only the Pinhole projection has a normalized image plane to distort, so the panorama
 * projections only accept DistortionModel::NoDistortion.
 */
[[nodiscard]] constexpr bool isValidCameraModel(CameraProjectionModel projection, DistortionModel distortion) {
    return projection == CameraProjectionModel::Pinhole || distortion == DistortionModel::NoDistortion;
}

/**
 * Combine a projection and a distortion model into the CameraModel calibrations store.
 * A Pinhole projection without distortion maps to CameraModel::Perspective, since Perspective with all-zero coefficients is the same camera.
 * @throws std::invalid_argument if the pair is not valid, see isValidCameraModel()
 */
[[nodiscard]] inline CameraModel toCameraModel(CameraProjectionModel projection, DistortionModel distortion) {
    if(!isValidCameraModel(projection, distortion)) {
        throw std::invalid_argument(std::string("The ") + std::string(toString(projection)) + " projection cannot carry the " + std::string(toString(distortion))
                                    + " distortion model, only Pinhole projections can be distorted");
    }
    switch(projection) {
        case CameraProjectionModel::Equirectangular:
            return CameraModel::Equirectangular;
        case CameraProjectionModel::Cylindrical:
            return CameraModel::Cylindrical;
        case CameraProjectionModel::Pinhole:
            break;
    }
    switch(distortion) {
        case DistortionModel::NoDistortion:
        case DistortionModel::BrownConrady:
            return CameraModel::Perspective;
        case DistortionModel::KannalaBrandt:
            return CameraModel::Fisheye;
        case DistortionModel::RadialDivision:
            return CameraModel::RadialDivision;
    }
    return CameraModel::Perspective;
}

}  // namespace dai
