#pragma once

#include <cstdint>
#include <string_view>

namespace dai {
/**
 * Camera model of an image: the projection surface the image is rendered on, together with the lens distortion applied on that surface.
 *
 * There is deliberately no separate enum for the projection and for the distortion. One value fixes both at once, because only the pinhole plane
 * carries distortion in depthai and every supported combination has a name of its own:
 *
 *  | CameraModel     | Projection surface | Distortion model (parametrized by the distortion coefficients)                         |
 *  |-----------------|--------------------|----------------------------------------------------------------------------------------|
 *  | Perspective     | pinhole plane      | Brown-Conrady, OpenCV layout (k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, taux, tauy) |
 *  | Fisheye         | pinhole plane      | Kannala-Brandt, OpenCV fisheye layout (k1, k2, k3, k4)                                 |
 *  | RadialDivision  | pinhole plane      | radial division (not evaluated by depthai-core yet)                                    |
 *  | Equirectangular | sphere             | none                                                                                   |
 *  | Cylindrical     | cylinder           | none                                                                                   |
 *
 * The projection surface tells how a direction (x, y, z) in the camera frame maps to normalized coordinates, and therefore what the intrinsic matrix
 * (fx, fy, cx, cy) means. The distortion model tells which formula warps those normalized coordinates on the pinhole plane before the intrinsics are
 * applied. Empty or all-zero coefficients mean an undistorted image for any of the pinhole models, so `Perspective` with no coefficients is a plain
 * pinhole camera.
 *
 * Pinhole models (`Perspective`, `Fisheye`, `RadialDivision`): the direction is first projected onto the plane z = 1, (x / z, y / z), the distortion of
 * the model is applied to that point, and the result is mapped to a pixel with u = fx * x' + cx, v = fy * y' + cy. fx and fy are the focal length in
 * pixels, (cx, cy) the principal point.
 *
 * Panorama models (`Equirectangular`, `Cylindrical`): these are the surfaces the Stitching node renders panoramas onto. They carry no distortion
 * coefficients, and their intrinsic matrix maps the angular coordinates of the direction to a pixel instead of the normalized image plane:
 *  - Equirectangular: u = fx * atan2(x, z) + cx, v = fy * asin(y / sqrt(x^2 + y^2 + z^2)) + cy
 *  - Cylindrical: u = fx * atan2(x, z) + cx, v = fy * y / sqrt(x^2 + z^2) + cy
 *
 * so fx and fy are the radius of the sphere or cylinder in pixels, and (cx, cy) is the pixel the camera Z axis projects to.
 *
 * ImgTransformation stores this value as its "distortion model" (getDistortionModel() / setDistortionModel()). That name predates the panorama
 * models; the field describes the whole camera model of the image as listed above, not only its distortion.
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
