#pragma once

#include <string>

#include "depthai/common/ImuParameters.hpp"

namespace dai {

// Get IMU noise parameters for the provided IMU name.
const ImuNoiseParameters& getImuNoiseParams(const std::string& imuName);

// Get the default IMU noise parameters
const ImuNoiseParameters& getDefaultImuNoiseParams();

}
