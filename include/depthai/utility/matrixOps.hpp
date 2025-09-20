#pragma once
#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <vector>

#include "depthai/utility/export.hpp"

namespace dai {
namespace matrix {

DEPTHAI_API std::vector<std::vector<float>> matMul(std::vector<std::vector<float>>& firstMatrix, std::vector<std::vector<float>>& secondMatrix);
DEPTHAI_API bool matInv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse);
DEPTHAI_API std::vector<std::vector<float>> createRotationMatrix(float theta);
DEPTHAI_API std::vector<std::vector<float>> createScalingMatrix(float scaleX, float scaleY);
DEPTHAI_API std::vector<std::vector<float>> createTranslationMatrix(float dx, float dy);
DEPTHAI_API std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
DEPTHAI_API std::vector<std::vector<float>> rvecToRotationMatrix(const double rvec[3]);
DEPTHAI_API void printMatrix(std::vector<std::vector<float>>& matrix);

}  // namespace matrix
}  // namespace dai