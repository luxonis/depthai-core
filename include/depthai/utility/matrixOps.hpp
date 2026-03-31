#pragma once
#include "depthai/common/Point3f.hpp"
#define _USE_MATH_DEFINES

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
#endif
namespace dai {
namespace matrix {

constexpr float MATRIX_EQ_EPSILON = 1e-6f;

std::array<std::array<float, 3>, 3> matMul(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B);
std::vector<std::vector<float>> matMul(const std::vector<std::vector<float>>& A, const std::vector<std::vector<float>>& B);
std::array<std::array<float, 4>, 4> matMul(const std::array<std::array<float, 4>, 4>& A, const std::array<std::array<float, 4>, 4>& B);

std::vector<float> matVecMul(const std::vector<std::vector<float>>& matrix, const std::vector<float>& vec);
std::array<float, 3> matVecMul(const std::array<std::array<float, 3>, 3>& matrix, const std::array<float, 3>& vec);

bool mateq(const std::vector<std::vector<float>>& A, const std::vector<std::vector<float>>& B, float epsilon = MATRIX_EQ_EPSILON);
bool mateq(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B, float epsilon = MATRIX_EQ_EPSILON);

bool matInv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse);

std::array<std::array<float, 2>, 2> getMatrixInverse(const std::array<std::array<float, 2>, 2>& matrix);
std::array<std::array<float, 3>, 3> getMatrixInverse(const std::array<std::array<float, 3>, 3>& matrix);

std::vector<std::vector<float>> createRotationMatrix(float theta);
std::vector<std::vector<float>> createScalingMatrix(float scaleX, float scaleY);
std::vector<std::vector<float>> createTranslationMatrix(float dx, float dy);
std::array<std::array<float, 4>, 4> createTransformationMatrix(const std::vector<std::vector<float>>& rotation, const dai::Point3f& translation);
std::array<std::array<float, 4>, 4> createTransformationMatrix(const std::array<std::array<float, 3>, 3>& rotation, const dai::Point3f& translation);
dai::Point3f transformPoint3f(const std::array<std::array<float, 4>, 4>& matrix, const dai::Point3f& point);
std::array<float, 4> dehomogenizePoint4(const std::array<float, 4>& point);
std::array<float, 3> dehomogenizePoint3(const std::array<float, 3>& point);

std::vector<float> matrixToVector(const std::vector<std::vector<float>>& R);
std::vector<float> matrix3x3ToVector(const std::array<std::array<float, 3>, 3>& R);
std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
std::vector<std::vector<float>> matrix3x3ToVectorMatrix(const std::array<std::array<float, 3>, 3>& R);
std::array<std::array<float, 3>, 3> vectorMatrixToMatrix3x3(const std::vector<std::vector<float>>& R);

std::array<std::array<float, 3>, 3> getRotationMatrixFromProjection4x4(const std::array<std::array<float, 4>, 4>& projection);

std::vector<std::vector<float>> rvecToRotationMatrix(const double rvec[3]);
void invertSe3Matrix4x4InPlace(std::vector<std::vector<float>>& mat);
std::array<std::array<float, 4>, 4> invertSe3Matrix4x4(const std::array<std::array<float, 4>, 4>& matrix);

void printMatrix(std::vector<std::vector<float>>& matrix);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

std::array<std::array<float, 3>, 3> cvMatToMatrix3x3(const cv::Mat& cvMat);

std::array<std::array<float, 4>, 4> cvMatToMatrix4x4(const cv::Mat& cvMat);

cv::Mat matrix3x3ToCvMat(const std::array<std::array<float, 3>, 3>& matrix);

cv::Mat matrix4x4ToCvMat(const std::array<std::array<float, 4>, 4>& matrix);

#endif

}  // namespace matrix
}  // namespace dai
