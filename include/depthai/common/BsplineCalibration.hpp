#pragma once

#include <array>
#include <vector>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Lens model of one camera: cubic splined-stereographic (mrcal LENSMODEL_SPLINED_STEREOGRAPHIC, order 3), fitted at
 * the native resolution width x height. It describes only the lens; the camera's position comes from the extrinsics
 * of the calibration, as for the OpenCV model.
 * intrinsics: fx, fy, cx, cy, then the (x, y) correction of each of the nx * ny control points, x varying fastest.
 */
struct BsplineCameraCalibration {
    int width = 0, height = 0;
    int nx = 0, ny = 0;
    double fovXDegrees = 0;
    std::vector<double> intrinsics;

    /**
     * Throws std::invalid_argument if the resolution, grid size, field of view or parameters are invalid.
     */
    void validate() const;

    /**
     * Project a ray in camera coordinates to a pixel, as mrcal 2.5.2 does, including its extrapolation outside
     * the grid. Call validate() once before projecting.
     */
    std::array<double, 2> project(const std::array<double, 3>& ray) const;

    /**
     * Unit ray in camera coordinates whose projection is the pixel: Newton iterations with the exact Jacobian of
     * the spline. Throws std::runtime_error if it does not converge to 1e-6 px. Call validate() once before.
     */
    std::array<double, 3> unproject(const std::array<double, 2>& pixel) const;

    DEPTHAI_SERIALIZE_DECL(BsplineCameraCalibration, width, height, nx, ny, fovXDegrees, intrinsics);
};
}  // namespace dai
