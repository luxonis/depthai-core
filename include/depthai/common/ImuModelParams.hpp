#pragma once

#include <string>

#include "depthai/utility/Serialization.hpp"

namespace dai {

struct AccelAxisNoiseParams {
    double vrw = 0.0; // velocity random walk
    double rrw = 0.0; // rate random walk
    double bi = 0.0;  // bias instability

    DEPTHAI_SERIALIZE(AccelAxisNoiseParams, vrw, rrw, bi);
};

struct GyroAxisNoiseParams {
    double arw = 0.0; // angle random walk
    double rrw = 0.0; // rate random walk
    double bi = 0.0;  // bias instability

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
