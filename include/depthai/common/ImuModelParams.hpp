#pragma once

#include <vector>

#include "depthai/utility/Serialization.hpp"

namespace dai {

struct AccelAxisNoiseParams {
    double vrw; // velocity random walk
    double rrw; // rate random walk
    double bi;  // bias instability

    DEPTHAI_SERIALIZE(AccelAxisNoiseParams, vrw, rrw, bi);
};

struct GyroAxisNoiseParams {
    double arw; // angle random walk
    double rrw; // rate random walk
    double bi;  // bias instability

    DEPTHAI_SERIALIZE(GyroAxisNoiseParams, arw, rrw, bi);
};

struct AccelerometerNoiseParams {
    AccelAxisNoiseParams x;
    AccelAxisNoiseParams y;
    AccelAxisNoiseParams z;

    DEPTHAI_SERIALIZE(AccelerometerNoiseParams, x, y, z);
};

struct GyroscopeNoiseParams {
    GyroAxisNoiseParams x;
    GyroAxisNoiseParams y;
    GyroAxisNoiseParams z;

    DEPTHAI_SERIALIZE(GyroscopeNoiseParams, x, y, z);
};

/// Calibration parameters IMU type specific, such as noise parameters, etc.
struct ImuModelParams {
    // IMU type name, e.g. "LSM6", "BMX160", etc.
    std::string name;

    // Accelerometer parameters
    AccelerometerNoiseParams accelerometer;
    
    // Gyroscope parameters
    GyroscopeNoiseParams gyroscope;

    DEPTHAI_SERIALIZE(ImuModelParams, name, accelerometer, gyroscope);
};

}  // namespace dai
