#include "depthai/utility/matrixOps.hpp"

#include <stdexcept>

namespace dai::matrix {

std::array<std::array<float, 3>, 3> cvMatToMatrix3x3(const cv::Mat& cvMat) {
    if(cvMat.rows != 3 || cvMat.cols != 3) {
        throw std::invalid_argument("Expected a 3x3 cv::Mat to convert to 3x3 matrix.");
    }
    std::array<std::array<float, 3>, 3> matrix;
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            matrix[i][j] = cvMat.at<float>(i, j);
        }
    }
    return matrix;
}

std::array<std::array<float, 4>, 4> cvMatToMatrix4x4(const cv::Mat& cvMat) {
    if(cvMat.rows != 4 || cvMat.cols != 4) {
        throw std::invalid_argument("Expected a 4x4 cv::Mat to convert to 4x4 matrix.");
    }
    std::array<std::array<float, 4>, 4> matrix;
    for(size_t i = 0; i < 4; ++i) {
        for(size_t j = 0; j < 4; ++j) {
            matrix[i][j] = cvMat.at<float>(i, j);
        }
    }
    return matrix;
}

cv::Mat matrix3x3ToCvMat(const std::array<std::array<float, 3>, 3>& matrix) {
    cv::Mat cvMat(3, 3, CV_32F);
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            cvMat.at<float>(i, j) = matrix[i][j];
        }
    }
    return cvMat;
}

cv::Mat matrix4x4ToCvMat(const std::array<std::array<float, 4>, 4>& matrix) {
    cv::Mat cvMat(4, 4, CV_32F);
    for(size_t i = 0; i < 4; ++i) {
        for(size_t j = 0; j < 4; ++j) {
            cvMat.at<float>(i, j) = matrix[i][j];
        }
    }
    return cvMat;
}

}  // namespace dai::matrix
