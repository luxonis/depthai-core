#pragma once

#include <memory>
#include <string>

#include <spdlog/logger.h>

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/ImgTransformations.hpp"

namespace dai {
namespace gate {

/**
 * Stamps a locally-calibrated image transformation with the current device
 * identity and, when a graph is installed, rebases it to the multi-device
 * component origin.
 *
 * When the local calibration no longer matches the graph the local metadata is
 * kept and a warning is logged once per `warnedForMismatch` flag.
 *
 * @param multiDeviceCalibration Resolved handler, or nullptr when no graph is set
 */
ImgTransformation prepareMultiDeviceTransformation(ImgTransformation transformation,
                                                   const std::string& deviceId,
                                                   const beta::MultiDeviceCalibrationHandler* multiDeviceCalibration,
                                                   bool& warnedForMismatch,
                                                   const std::shared_ptr<spdlog::logger>& logger);

}  // namespace gate
}  // namespace dai
