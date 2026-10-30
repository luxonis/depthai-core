#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/CalibrationHandler.hpp"

namespace dai {
namespace detail {

using StereoPairTransform = std::vector<std::vector<float>>;

inline std::optional<float> stereoPairPositionDeltaInView(const StereoPairTransform& firstToCommon, const StereoPairTransform& secondToCommon, bool vertical) {
    using Vector3 = std::array<float, 3>;
    static constexpr float epsilon = 1e-6f;

    const auto validTransform = [](const StereoPairTransform& transform) {
        if(transform.size() != 4) return false;
        for(const auto& row : transform) {
            if(row.size() != 4) return false;
        }
        return true;
    };
    if(!validTransform(firstToCommon) || !validTransform(secondToCommon)) return std::nullopt;

    const auto dot = [](const Vector3& first, const Vector3& second) { return first[0] * second[0] + first[1] * second[1] + first[2] * second[2]; };
    const auto normalized = [&dot](const Vector3& vector) -> std::optional<Vector3> {
        const auto length = std::sqrt(dot(vector, vector));
        if(length <= epsilon) return std::nullopt;
        return Vector3{vector[0] / length, vector[1] / length, vector[2] / length};
    };

    // The third rotation column is the camera's optical axis expressed in the common frame.
    const auto forward =
        normalized({firstToCommon[0][2] + secondToCommon[0][2], firstToCommon[1][2] + secondToCommon[1][2], firstToCommon[2][2] + secondToCommon[2][2]});
    if(!forward) return std::nullopt;

    // Keep the common frame's down direction, but remove any component along the optical axis.
    // This makes the derived right direction reverse naturally for a backward-facing pair.
    constexpr Vector3 commonDown{0.0f, 1.0f, 0.0f};
    const auto downAlongForward = dot(commonDown, *forward);
    const auto down = normalized(
        {commonDown[0] - downAlongForward * (*forward)[0], commonDown[1] - downAlongForward * (*forward)[1], commonDown[2] - downAlongForward * (*forward)[2]});
    if(!down) return std::nullopt;

    const Vector3 right{(*down)[1] * (*forward)[2] - (*down)[2] * (*forward)[1],
                        (*down)[2] * (*forward)[0] - (*down)[0] * (*forward)[2],
                        (*down)[0] * (*forward)[1] - (*down)[1] * (*forward)[0]};
    const Vector3 positionDelta{
        secondToCommon[0][3] - firstToCommon[0][3], secondToCommon[1][3] - firstToCommon[1][3], secondToCommon[2][3] - firstToCommon[2][3]};
    const auto projectedDelta = dot(positionDelta, vertical ? *down : right);
    if(std::abs(projectedDelta) <= epsilon) return std::nullopt;
    return projectedDelta;
}

inline bool stereoPairFirstCameraIsLeft(
    const CalibrationHandler& calibrationHandler, CameraBoardSocket first, CameraBoardSocket second, bool vertical, float baseline) {
    // Legacy getStereoPairs() inferred left/right solely from the sign of the pairwise X/Y translation. That ordering is reversed for cameras
    // which face backward relative to the calibration origin. Preserve the legacy rule as a fallback when disconnected or degenerate calibration
    // data prevents both cameras from being compared in a common viewing frame.
    const bool legacyOrder = baseline < 0.0f;

    try {
        CameraBoardSocket firstRoot = CameraBoardSocket::AUTO;
        CameraBoardSocket secondRoot = CameraBoardSocket::AUTO;
        const auto firstToRoot = calibrationHandler.getExtrinsicsToOrigin(first, false, firstRoot);
        const auto secondToRoot = calibrationHandler.getExtrinsicsToOrigin(second, false, secondRoot);
        if(firstRoot != secondRoot) return legacyOrder;

        const auto commonFrameDelta = stereoPairPositionDeltaInView(firstToRoot, secondToRoot, vertical);
        return commonFrameDelta ? *commonFrameDelta > 0.0f : legacyOrder;
    } catch(const std::exception&) {
        return legacyOrder;
    }
}

class StereoPairCalculator {
   public:
    static std::vector<StereoPair> find(const CalibrationHandler& calibrationHandler, const std::vector<CameraFeatures>& connectedFeatures) {
        const float pi = std::acos(-1.0f);
        std::vector<StereoPair> stereoPairs;
        std::unordered_map<CameraBoardSocket, CameraFeatures> featureBySocket;
        std::vector<CameraBoardSocket> sockets;
        sockets.reserve(connectedFeatures.size());

        const auto isStereoCapable = [](const CameraFeatures& feature) {
            return std::find(feature.supportedTypes.begin(), feature.supportedTypes.end(), CameraSensorType::COLOR) != feature.supportedTypes.end()
                   || std::find(feature.supportedTypes.begin(), feature.supportedTypes.end(), CameraSensorType::MONO) != feature.supportedTypes.end();
        };

        for(const auto& feature : connectedFeatures) {
            if(!isStereoCapable(feature)) continue;
            featureBySocket.emplace(feature.socket, feature);
            sockets.push_back(feature.socket);
        }

        for(size_t i = 0; i < sockets.size(); ++i) {
            const auto first = sockets[i];
            const auto& firstFeature = featureBySocket.at(first);
            if(!calibrationHandler.hasCameraCalibration(first)) continue;
            const float firstFov = calibrationHandler.getFov(first, false);

            for(size_t j = i + 1; j < sockets.size(); ++j) {
                const auto second = sockets[j];
                const auto& secondFeature = featureBySocket.at(second);
                if(!calibrationHandler.hasCameraCalibration(second)) continue;
                if(!calibrationHandler.checkExtrinsicsLink(first, second) && !calibrationHandler.checkExtrinsicsLink(second, first)) continue;
                const float secondFov = calibrationHandler.getFov(second, false);
                if(firstFeature.sensorName != secondFeature.sensorName) continue;
                float maximalAngle = std::min(firstFov, secondFov) * pi / 180.0f * 0.5f;
                if(maximalAngle == 0.0f) maximalAngle = pi / 4.0f;
                if(calibrationHandler.getCameraZAxisAngle(first, second) > maximalAngle) continue;

                const auto translation = calibrationHandler.getCameraTranslationVector(first, second, false);
                const auto ax = std::abs(translation[0]);
                const auto ay = std::abs(translation[1]);
                const auto az = std::abs(translation[2]);
                const bool vertical = ax < ay;
                if(std::max(ax, ay) < az) continue;

                const float baseline = vertical ? translation[1] : translation[0];
                const bool firstIsLeft = stereoPairFirstCameraIsLeft(calibrationHandler, first, second, vertical, baseline);
                StereoPair pair;
                pair.left = firstIsLeft ? first : second;
                pair.right = firstIsLeft ? second : first;
                pair.baseline = std::abs(baseline);
                pair.isVertical = vertical;
                stereoPairs.push_back(pair);
            }
        }

        std::sort(stereoPairs.begin(), stereoPairs.end(), [](const StereoPair& first, const StereoPair& second) { return first.baseline > second.baseline; });
        return stereoPairs;
    }
};

}  // namespace detail
}  // namespace dai
