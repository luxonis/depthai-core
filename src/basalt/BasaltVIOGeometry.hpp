#pragma once

#include <stdexcept>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/matrixOps.hpp"

namespace dai {
namespace node {
namespace detail {
// Basalt currently estimates the physical first camera using CalibrationHandler.
// Its API pose uses that camera's origin and FLU axes, even if metadata names a
// different calibration reference. Do not include the world-frame localTransform.
inline Extrinsics getBasaltTrackedFrameExtrinsics(const CalibrationHandler& calibration, const ImgFrame& firstFrame, const std::string& deviceId) {
    const auto inputExtrinsics = firstFrame.transformation.getExtrinsics();
    if(!inputExtrinsics.toDeviceId.empty() && inputExtrinsics.toDeviceId != deviceId) {
        throw std::invalid_argument("BasaltVIO: input metadata references another device's calibration frame");
    }
    const auto reference = inputExtrinsics.toCameraSocket == CameraBoardSocket::AUTO ? calibration.getCameraWithLowestId() : inputExtrinsics.toCameraSocket;
    const auto camera = static_cast<CameraBoardSocket>(firstFrame.getInstanceNum());
    const auto cameraToReference = calibration.getCameraExtrinsics(camera, reference, false, LengthUnit::METER);
    const std::vector<std::vector<float>> fluToOptical{{0, -1, 0, 0}, {0, 0, -1, 0}, {1, 0, 0, 0}, {0, 0, 0, 1}};
    Extrinsics result(matrix::matMul(cameraToReference, fluToOptical), reference, LengthUnit::METER);
    // Retain the same device identity convention as the image metadata (including
    // unknown/empty on older firmware); never invent a different reference frame.
    result.toDeviceId = inputExtrinsics.toDeviceId;
    return result;
}
}  // namespace detail
}  // namespace node
}  // namespace dai
