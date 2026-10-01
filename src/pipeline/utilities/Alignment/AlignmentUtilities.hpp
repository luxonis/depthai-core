#pragma once
#include <assert.h>
#include <spdlog/async_logger.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <depthai/utility/matrixOps.hpp>
#include <vector>

#include "depthai/common/CameraModel.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/matrixOps.hpp"

/**
 * Turn a direction in camera space into a pixel coordinate on the source image of the transformation, following its projection and distortion models.
 * @throws std::runtime_error if the direction cannot be projected, see projectDirection()
 */
dai::Point2f rayToPixel(const std::array<float, 3>& ray, const dai::ImgTransformation& transformation);

/**
 * Turn a pixel coordinate on the source image of a Pinhole transformation into a 3D ray in camera space normalized to z = 1, applying undistortion if
 * necessary.
 * @throws std::invalid_argument for a panorama projection, whose pixels do not map to the normalized image plane
 */
std::array<float, 3> pixelToRay(dai::Point2f px, const dai::ImgTransformation& transformation);

/**
 * Project a direction in camera space onto the image coordinates an intrinsic matrix turns into a pixel, following the projection model (see
 * dai::CameraProjectionModel) and, for the Pinhole projection, distorting the normalized image plane coordinates.
 * @return Homogeneous image coordinates (x, y, 1)
 * @throws std::runtime_error if the direction cannot be projected: behind a Pinhole camera (z <= 0), or on the axis of a Cylindrical panorama
 */
std::array<float, 3> projectDirection(const std::array<float, 3>& direction,
                                      dai::CameraProjectionModel projection,
                                      dai::DistortionModel distortion,
                                      const std::vector<float>& coeffs);

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
 * Distort a point on the normalized image plane using the specified distortion model and coefficients.
 */
std::array<float, 3> distortPoint(std::array<float, 3> point, dai::DistortionModel model, const std::vector<float>& coeffs);

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
 * Undistort a point on the normalized image plane using the specified distortion model and coefficients.
 */
std::array<float, 3> undistortPoint(std::array<float, 3> point, dai::DistortionModel model, const std::vector<float>& coeffs);

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
