#pragma once
#include <assert.h>
#include <spdlog/async_logger.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
#endif

#include <utility/matrixOps.hpp>

#include "common/CameraModel.hpp"
#include "common/Point2f.hpp"
#include "depthai/common/CameraModel.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/matrixOps.hpp"

/** TODO: draw a diagram of how a point can be manipulated going from a 3D point, to a pixel in a cropped frame to a pixel in another frame with different
 * intrinsics, distortions and rotation of camera  (== rectification)
 *
 */

//
//
//
// Distortions
//
//
//

/**
 * Distort a point using perspective distortion coefficients.
 */
std::array<float, 3> distortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs);

/**
 * Distort a point using fisheye distortion coefficients.
 */
std::array<float, 3> distortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs);

/**
 * Distort a point using radial division distortion coefficients.
 */
std::array<float, 3> distortRadialDivision(std::array<float, 3> point, const std::vector<float>& coeffs);

/**
 * Apply tilt to a point.
 */
std::array<float, 3> applyTilt(float x, float y, float tauX, float tauY);

/**
 * Distort a point using the specified camera model and distortion coefficients.
 */
std::array<float, 3> distortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs);

//
//
//
// Undistortions
//
//
//

/**
 * Undistort a point using perspective distortion coefficients.
 */
std::array<float, 3> undistortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs);

/**
 * Undistort a point using fisheye distortion coefficients.
 */
std::array<float, 3> undistortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs);

/**
 * Undistort a point using radial division distortion coefficients.
 */
std::array<float, 3> undistortRadialDivision(std::array<float, 3> point, const std::vector<float>& coeffs);

/**
 * Undistort a point using the specified camera model and distortion coefficients.
 */
std::array<float, 3> undistortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs);

//
//
//
// General
//
//
//

/**
 * Check if the distortion coefficients have any non-zero values.
 * @param coeffs Distortion coefficients to check
 * @return true if any coefficient has a non-zero value, false otherwise
 */
bool hasNonZeroDistortion(const std::vector<float>& coeffs);

/**
 * Get the distortion coefficient at the specified index, or 0 if the index is out of range.
 */
float coeffAt(const std::vector<float>& coeffs, size_t idx);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
/**
 * Transform a point from one transformation to another using OpenCV functions. This is used when the transformations have different distortion coefficients, as
 * the OpenCV functions can handle the distortion and undistortion in one step, which is more accurate than doing it separately.
 */
dai::Point2f opencvPointTransformation(dai::Point2f sourcePt, const dai::ImgTransformation& from, const dai::ImgTransformation& to);
#endif