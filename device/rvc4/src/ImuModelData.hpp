#pragma once

#include <string>

#include "depthai/common/ImuParameters.hpp"

namespace dai {

const ImuNoiseParameters& getImuNoiseParams(const std::string& imuName);

const ImuNoiseParameters& getDefaultImuNoiseParams();
}
