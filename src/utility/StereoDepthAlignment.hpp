#pragma once

#include <cmath>
#include <stdexcept>

#include "depthai/common/StereoPair.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {
namespace utility {

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

}  // namespace utility
}  // namespace dai
