#pragma once

#include <memory>
#include <string>

#include <spdlog/logger.h>

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/ImgTransformations.hpp"

namespace dai {
namespace gate {

ImgTransformation prepareMultiDeviceTransformation(ImgTransformation transformation,  // multiDeviceCalibration: nullptr when no graph is set
                                                   const std::string& deviceId,
                                                   const beta::MultiDeviceCalibrationHandler* multiDeviceCalibration,
                                                   bool& warnedForMismatch,
                                                   const std::shared_ptr<spdlog::logger>& logger);

}  // namespace gate
}  // namespace dai
