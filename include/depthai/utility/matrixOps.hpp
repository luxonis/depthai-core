#pragma once
#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <vector>

namespace dai {
namespace matrix {

std::vector<std::vector<float>> matMul(std::vector<std::vector<float>>& firstMatrix, std::vector<std::vector<float>>& secondMatrix);
bool matInv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse);
std::vector<std::vector<float>> createRotationMatrix(float theta);
std::vector<std::vector<float>> createScalingMatrix(float scaleX, float scaleY);
std::vector<std::vector<float>> createTranslationMatrix(float dx, float dy);
std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
std::vector<std::vector<float>> rvecToRotationMatrix(const double rvec[3]);
void printMatrix(std::vector<std::vector<float>>& matrix);

}  // namespace matrix
}  // namespace dai