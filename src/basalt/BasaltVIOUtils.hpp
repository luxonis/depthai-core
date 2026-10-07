#pragma once

#ifndef SOPHUS_USE_BASIC_LOGGING
    #define SOPHUS_USE_BASIC_LOGGING
#endif

#include "basalt/calibration/calibration.hpp"
#include "basalt/utils/vio_config.h"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace utility {

// Shared by host BasaltVIO and RVC4 firmware VIO.
basalt::GenericCamera<double> getBasaltCameraModel(const CalibrationHandler& calibration, const ImgFrame& frame);
basalt::VioConfig getDefaultBasaltVIOConfig();

}  // namespace utility
}  // namespace dai
