#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "depthai/common/Point3f.hpp"

namespace dai {

constexpr float kMatrixIdentityTolerance = 1e-6f;

inline const std::vector<std::vector<float>> kIdentityRotationMatrix = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

inline bool is3x3Matrix(const std::vector<std::vector<float>>& matrix) {
    return matrix.size() == 3 && matrix[0].size() == 3 && matrix[1].size() == 3 && matrix[2].size() == 3;
}

inline bool is3x4CalibrationMatrix(const std::vector<std::vector<float>>& calibration) {
    return calibration.size() == 3 && calibration[0].size() >= 4 && calibration[1].size() >= 4 && calibration[2].size() >= 4;
}

inline void apply3x4CalibrationMatrix(const std::vector<std::vector<float>>& calibration, float& x, float& y, float& z) {
    if(!is3x4CalibrationMatrix(calibration)) {
        return;
    }

    const std::array<float, 3> input = {x, y, z};

    x = calibration[0][0] * input[0] + calibration[0][1] * input[1] + calibration[0][2] * input[2] + calibration[0][3];
    y = calibration[1][0] * input[0] + calibration[1][1] * input[1] + calibration[1][2] * input[2] + calibration[1][3];
    z = calibration[2][0] * input[0] + calibration[2][1] * input[1] + calibration[2][2] * input[2] + calibration[2][3];
}

inline void apply3x3Matrix(const std::vector<std::vector<float>>& matrix, float& x, float& y, float& z) {
    if(!is3x3Matrix(matrix)) {
        return;
    }

    const std::array<float, 3> input = {x, y, z};

    x = matrix[0][0] * input[0] + matrix[0][1] * input[1] + matrix[0][2] * input[2];
    y = matrix[1][0] * input[0] + matrix[1][1] * input[1] + matrix[1][2] * input[2];
    z = matrix[2][0] * input[0] + matrix[2][1] * input[1] + matrix[2][2] * input[2];
}

inline void normalizeQuaternion(float& x, float& y, float& z, float& w) {
    const float norm = std::sqrt(x * x + y * y + z * z + w * w);
    if(norm <= 1e-9f) {
        return;
    }

    const float invNorm = 1.0f / norm;
    x *= invNorm;
    y *= invNorm;
    z *= invNorm;
    w *= invNorm;
}

inline void quaternionMultiply(float ax, float ay, float az, float aw, float bx, float by, float bz, float bw, float& x, float& y, float& z, float& w) {
    x = aw * bx + ax * bw + ay * bz - az * by;
    y = aw * by - ax * bz + ay * bw + az * bx;
    z = aw * bz + ax * by - ay * bx + az * bw;
    w = aw * bw - ax * bx - ay * by - az * bz;
}

inline void matrixToQuaternion(const std::vector<std::vector<float>>& matrix, float& x, float& y, float& z, float& w) {
    if(!is3x3Matrix(matrix)) {
        return;
    }

    const float trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if(trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (matrix[2][1] - matrix[1][2]) / s;
        y = (matrix[0][2] - matrix[2][0]) / s;
        z = (matrix[1][0] - matrix[0][1]) / s;
    } else if(matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
        const float s = std::sqrt(1.0f + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0f;
        w = (matrix[2][1] - matrix[1][2]) / s;
        x = 0.25f * s;
        y = (matrix[0][1] + matrix[1][0]) / s;
        z = (matrix[0][2] + matrix[2][0]) / s;
    } else if(matrix[1][1] > matrix[2][2]) {
        const float s = std::sqrt(1.0f + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0f;
        w = (matrix[0][2] - matrix[2][0]) / s;
        x = (matrix[0][1] + matrix[1][0]) / s;
        y = 0.25f * s;
        z = (matrix[1][2] + matrix[2][1]) / s;
    } else {
        const float s = std::sqrt(1.0f + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0f;
        w = (matrix[1][0] - matrix[0][1]) / s;
        x = (matrix[0][2] + matrix[2][0]) / s;
        y = (matrix[1][2] + matrix[2][1]) / s;
        z = 0.25f * s;
    }

    normalizeQuaternion(x, y, z, w);
}

inline bool isIdentityRotationMatrix(const std::vector<std::vector<float>>& rotationMatrix, float tolerance = kMatrixIdentityTolerance) {
    if(!is3x3Matrix(rotationMatrix)) {
        return false;
    }

    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t col = 0; col < 3; ++col) {
            const float expected = row == col ? 1.0f : 0.0f;
            if(std::fabs(rotationMatrix[row][col] - expected) > tolerance) {
                return false;
            }
        }
    }

    return true;
}

inline bool isZeroRotationMatrix(const std::vector<std::vector<float>>& matrix, float tolerance = kMatrixIdentityTolerance) {
    if(!is3x3Matrix(matrix)) {
        return false;
    }

    for(const auto& row : matrix) {
        for(const auto& value : row) {
            if(std::fabs(value) > tolerance) {
                return false;
            }
        }
    }

    return true;
}

inline bool isZeroTranslation(const dai::Point3f& translation) {
    return translation.x == 0.0f && translation.y == 0.0f && translation.z == 0.0f;
}

}  // namespace dai
