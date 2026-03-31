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
