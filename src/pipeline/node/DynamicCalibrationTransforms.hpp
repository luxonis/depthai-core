#pragma once

#include <depthai/common/StereoPair.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <depthai/utility/matrixOps.hpp>
#include <map>
#include <stdexcept>
#include <vector>

#include "../../utility/StereoDepthAlignment.hpp"

namespace dai {
namespace node {
namespace detail {

// All input poses transform the common base (housing, when available) into a camera.
// Every unobserved camera follows the first stereo pair's default depth reference camera through its factory transform.
inline CalibrationHandler assembleDynamicCalibration(const CalibrationHandler& current,
                                                     const CalibrationHandler& factory,
                                                     const std::vector<CameraBoardSocket>& sockets,
                                                     const std::map<CameraBoardSocket, std::vector<std::vector<float>>>& calibratedPoses,
                                                     bool housingBase,
                                                     const std::vector<StereoPair>& stereoPairs,
                                                     Platform platform) {
    if(calibratedPoses.empty()) throw std::invalid_argument("DynamicCalibration has no calibrated camera poses.");

    auto poses = calibratedPoses;
    for(auto socket : sockets) {
        if(!calibratedPoses.count(socket)) {
            if(stereoPairs.empty()) {
                throw std::invalid_argument("DynamicCalibration requires a stereo pair to anchor unobserved cameras to factory extrinsics.");
            }
            const auto anchor = utility::stereoDepthReferenceCamera(stereoPairs.front(), current, platform);
            if(!calibratedPoses.count(anchor)) {
                throw std::invalid_argument(
                    "DynamicCalibration requires the first stereo pair's default depth reference camera as a calibration input to anchor unobserved cameras to "
                    "factory "
                    "extrinsics.");
            }
            // Use measured factory translations, in the same units as DCL (meters).
            poses[socket] = matrix::matMul(factory.getCameraExtrinsics(anchor, socket, false, LengthUnit::METER), calibratedPoses.at(anchor));
        }
    }

    auto result = current;
    for(size_t idx = 0; idx + 1 < sockets.size(); ++idx) {
        auto cameraToBase = poses.at(sockets[idx]);
        matrix::invertSe3Matrix4x4InPlace(cameraToBase);
        const auto cameraToNext = matrix::matMul(poses.at(sockets[idx + 1]), cameraToBase);
        auto translation = matrix::extractTranslationVector(cameraToNext);
        for(auto& value : translation) value *= 100.0f;
        result.updateCameraExtrinsics(sockets[idx], sockets[idx + 1], matrix::extractRotationMatrix(cameraToNext), translation);
    }

    if(housingBase) {
        const auto& housingToOrigin = poses.at(current.getEepromData().housingExtrinsics.toCameraSocket);
        auto eeprom = result.getEepromData();
        eeprom.housingExtrinsics.rotationMatrix = matrix::extractRotationMatrix(housingToOrigin);
        // EEPROM extrinsics use centimeters; the DCL poses above use meters.
        eeprom.housingExtrinsics.translation = {100.0f * housingToOrigin[0][3], 100.0f * housingToOrigin[1][3], 100.0f * housingToOrigin[2][3]};
        result = CalibrationHandler(eeprom);
    }
    return result;
}

}  // namespace detail
}  // namespace node
}  // namespace dai
