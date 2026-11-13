#pragma once

#include <string>
#include <vector>

#include "depthai/utility/Serialization.hpp"

namespace dai {

struct AccelAxisNoiseParams {
    float noiseDensity = 0.0f;
    float randomWalk = 0.0f;
    float biasStability = 0.0f;
    DEPTHAI_SERIALIZE_DECL(AccelAxisNoiseParams, noiseDensity, randomWalk, biasStability);
};

struct GyroAxisNoiseParams {
    float noiseDensity = 0.0f;
    float randomWalk = 0.0f;
    float biasStability = 0.0f;
    DEPTHAI_SERIALIZE_DECL(GyroAxisNoiseParams, noiseDensity, randomWalk, biasStability);
};

struct AccelerometerNoiseParams {
    AccelAxisNoiseParams x;
    AccelAxisNoiseParams y;
    AccelAxisNoiseParams z;

    DEPTHAI_SERIALIZE_DECL(AccelerometerNoiseParams, x, y, z);
};

struct GyroscopeNoiseParams {
    GyroAxisNoiseParams x;
    GyroAxisNoiseParams y;
    GyroAxisNoiseParams z;

    DEPTHAI_SERIALIZE_DECL(GyroscopeNoiseParams, x, y, z);
};

/// Allan-variance-derived IMU noise parameters.
struct ImuNoiseParameters {
    // IMU type name, e.g. "LSM6", "BMX160", etc.
    std::string name;

    // Accelerometer parameters
    AccelerometerNoiseParams accelerometer;

    // Gyroscope parameters
    GyroscopeNoiseParams gyroscope;

    DEPTHAI_SERIALIZE_DECL(ImuNoiseParameters, name, accelerometer, gyroscope);
};

/// Complete IMU parameter payload: noise + per-sensor canonical affine calibration.
struct ImuCalibrationParams {
    ImuNoiseParameters noise;
    std::vector<std::vector<float>> accelerometer = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}};
    std::vector<std::vector<float>> gyroscope = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}};

    DEPTHAI_SERIALIZE_DECL(ImuCalibrationParams, noise, accelerometer, gyroscope);
};

}  // namespace dai
