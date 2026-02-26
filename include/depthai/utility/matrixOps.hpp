#pragma once
#define _USE_MATH_DEFINES

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

namespace dai {
namespace matrix {

std::array<std::array<float, 3>, 3> matMul(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B);
std::vector<std::vector<float>> matMul(const std::vector<std::vector<float>>& firstMatrix, const std::vector<std::vector<float>>& secondMatrix);
std::vector<float> matVecMul(const std::vector<std::vector<float>>& matrix, const std::vector<float>& vec);
std::array<float, 3> matVecMul(const std::array<std::array<float, 3>, 3>& matrix, const std::array<float, 3>& vec);
bool mateq(const std::vector<std::vector<float>>& A, const std::vector<std::vector<float>>& B);
bool mateq(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B);
bool matInv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse);
std::array<std::array<float, 2>, 2> getMatrixInverse(const std::array<std::array<float, 2>, 2>& matrix);
std::array<std::array<float, 3>, 3> getMatrixInverse(const std::array<std::array<float, 3>, 3>& matrix);
std::vector<std::vector<float>> createRotationMatrix(float theta);
std::vector<std::vector<float>> createScalingMatrix(float scaleX, float scaleY);
std::vector<std::vector<float>> createTranslationMatrix(float dx, float dy);
std::vector<float> matrixToVector(const std::vector<std::vector<float>>& R);
std::vector<float> matrix3x3ToVector(const std::array<std::array<float, 3>, 3>& R);
std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
std::vector<std::vector<float>> matrix3x3toVectorMatrix(const std::array<std::array<float, 3>, 3>& R);

std::array<std::array<float, 3>, 3> getRotationMatrixFromProjection4x4(const std::array<std::array<float, 4>, 4>& projection);

std::vector<std::vector<float>> rvecToRotationMatrix(const double rvec[3]);
std::vector<std::vector<float>> invertSe3Matrix4x4(const std::vector<std::vector<float>>& matrix);
void printMatrix(std::vector<std::vector<float>>& matrix);

}  // namespace matrix
}  // namespace dai
