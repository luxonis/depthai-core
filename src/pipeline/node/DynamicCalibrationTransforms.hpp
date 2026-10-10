#pragma once

#include <cmath>
#include <depthai/common/StereoPair.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <depthai/device/Platform.hpp>
#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>
#include <depthai/utility/matrixOps.hpp>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace dai {
namespace node {
namespace detail {

using StereoDepthAlign = StereoDepthConfig::AlgorithmControl::DepthAlign;

// Mirror the RVC4 firmware's CAM_A proximity policy for calibration anchoring.
// Keep AUTO unchanged on the wire: the firmware owns stereo output alignment.
inline StereoDepthAlign defaultStereoDepthAlignment(const StereoPair& pair, const CalibrationHandler& calibration, Platform platform) {
    if(platform == Platform::RVC4) {
        try {
            const auto left = calibration.getCameraTranslationVector(pair.left, CameraBoardSocket::CAM_A, false);
            const auto right = calibration.getCameraTranslationVector(pair.right, CameraBoardSocket::CAM_A, false);
            const auto distance = [](const std::vector<float>& t) { return std::sqrt(t.at(0) * t.at(0) + t.at(1) * t.at(1) + t.at(2) * t.at(2)) / 100.f; };
            if(distance(right) < distance(left) * 0.8f) return StereoDepthAlign::RECTIFIED_RIGHT;
        } catch(const std::exception&) {
            // Firmware also falls back to rectified-left when CAM_A extrinsics are unavailable.
        }
    }
    return StereoDepthAlign::RECTIFIED_LEFT;
}

inline CameraBoardSocket stereoDepthReferenceCamera(const StereoPair& pair,
                                                    const CalibrationHandler& calibration,
                                                    Platform platform,
                                                    StereoDepthAlign alignment = StereoDepthAlign::AUTO) {
    if(alignment == StereoDepthAlign::AUTO) alignment = defaultStereoDepthAlignment(pair, calibration, platform);
    switch(alignment) {
        case StereoDepthAlign::LEFT:
        case StereoDepthAlign::RECTIFIED_LEFT:
            return pair.left;
        case StereoDepthAlign::RIGHT:
        case StereoDepthAlign::RECTIFIED_RIGHT:
            return pair.right;
        case StereoDepthAlign::AUTO:
        case StereoDepthAlign::CENTER:
            throw std::invalid_argument("Stereo depth alignment has no single reference camera.");
    }
    throw std::invalid_argument("Invalid stereo depth alignment.");
}

// All input poses transform the common base (housing, when available) into a camera.
// Every unobserved camera follows an anchor camera through its factory transform. The anchor is the default depth
// reference camera of the first stereo pair (in device order) whose reference camera is a calibration input.
inline CalibrationHandler assembleDynamicCalibration(const CalibrationHandler& current,
                                                     const CalibrationHandler& factory,
                                                     const std::vector<CameraBoardSocket>& sockets,
                                                     const std::map<CameraBoardSocket, std::vector<std::vector<float>>>& calibratedPoses,
                                                     bool housingBase,
                                                     const std::vector<StereoPair>& stereoPairs,
                                                     Platform platform) {
    if(calibratedPoses.empty()) throw std::invalid_argument("DynamicCalibration has no calibrated camera poses.");

    auto poses = calibratedPoses;
    std::optional<CameraBoardSocket> anchor;
    for(const auto& stereoPair : stereoPairs) {
        const auto candidate = stereoDepthReferenceCamera(stereoPair, current, platform);
        if(calibratedPoses.count(candidate)) {
            anchor = candidate;
            break;
        }
    }
    for(auto socket : sockets) {
        if(!calibratedPoses.count(socket)) {
            if(stereoPairs.empty()) {
                throw std::invalid_argument("DynamicCalibration requires a stereo pair to anchor unobserved cameras to factory extrinsics.");
            }
            if(!anchor) {
                throw std::invalid_argument(
                    "DynamicCalibration requires a stereo pair whose default depth reference camera is a calibration input to anchor unobserved cameras to "
                    "factory extrinsics.");
            }
            // Use measured factory translations, in the same units as DCL (meters).
            poses[socket] = matrix::matMul(factory.getCameraExtrinsics(*anchor, socket, false, LengthUnit::METER), calibratedPoses.at(*anchor));
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
