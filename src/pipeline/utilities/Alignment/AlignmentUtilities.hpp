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

dai::Point2f rayToPixel(const std::array<float, 3>& ray, const dai::ImgTransformation& transformation);  // NaN for rays with z <= 0 (non-panorama models)

std::array<float, 3> pixelToRay(dai::Point2f px, const dai::ImgTransformation& transformation);

std::array<float, 3> distortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs);

std::array<float, 3> distortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs);

std::array<float, 3> distortRadialDivision(std::array<float, 3> point, const std::vector<float>& coeffs);

std::array<float, 3> applyTilt(float x, float y, float tauX, float tauY);

std::array<float, 3> distortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs);

std::array<float, 3> undistortPerspective(std::array<float, 3> point, const std::vector<float>& coeffs);

std::array<float, 3> undistortFisheye(std::array<float, 3> point, const std::vector<float>& coeffs);

std::array<float, 3> undistortRadialDivision(std::array<float, 3> point, const std::vector<float>& coeffs);

std::array<float, 3> undistortPoint(std::array<float, 3> point, dai::CameraModel model, const std::vector<float>& coeffs);

bool hasNonZeroDistortion(const std::vector<float>& coeffs);

float coeffAt(const std::vector<float>& coeffs, size_t idx);  // 0 if idx is out of range
