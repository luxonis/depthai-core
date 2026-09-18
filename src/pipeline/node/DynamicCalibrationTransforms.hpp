#pragma once

#include <array>
#include <cmath>
#include <depthai/common/StereoPair.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <depthai/utility/matrixOps.hpp>
#include <map>
#include <stdexcept>
#include <variant>
#include <vector>

#include "../../utility/StereoDepthAlignment.hpp"

namespace dai {
namespace node {
namespace detail {

// Pose of a camera in the frame DynamicCalibrationLibrary optimizes in: the housing when the board
// defines one, otherwise the calibration chain's base camera.
inline std::vector<std::vector<float>> baseToCameraTransform(const CalibrationHandler& calibration,
                                                             const std::variant<CameraBoardSocket, HousingCoordinateSystem>& base,
                                                             CameraBoardSocket socket) {
    if(const auto* cameraBase = std::get_if<CameraBoardSocket>(&base)) {
        if(*cameraBase == socket) {
            return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
        }
        return calibration.getCameraExtrinsics(*cameraBase, socket, false, LengthUnit::METER);
    }

    auto socketToHousingTransform = calibration.getHousingCalibration(socket, HousingCoordinateSystem::AUTO, false, LengthUnit::METER);
    matrix::invertSe3Matrix4x4InPlace(socketToHousingTransform);
    return socketToHousingTransform;
}

// Rotations here are composed, inverted and exponentiated repeatedly, and the result has to leave the
// untouched factory links bit-for-bit where they were, so this arithmetic stays in double.
using Rotation3d = std::array<std::array<double, 3>, 3>;

inline Rotation3d rotationOf(const std::vector<std::vector<float>>& transform) {
    Rotation3d rotation{};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) rotation[row][col] = transform[row][col];
    }
    return rotation;
}

inline Rotation3d multiply(const Rotation3d& left, const Rotation3d& right) {
    Rotation3d result{};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            for(int k = 0; k < 3; ++k) result[row][col] += left[row][k] * right[k][col];
        }
    }
    return result;
}

inline Rotation3d transposed(const Rotation3d& rotation) {
    Rotation3d result{};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) result[row][col] = rotation[col][row];
    }
    return result;
}

inline double rotationAngleDegrees(const Rotation3d& rotation) {
    const double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
    return std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0))) * 180.0 / M_PI;
}

// Poses are composed and inverted several times on the way to the stored extrinsics, and the links the
// re-anchoring does not touch have to come out exactly where they went in, so this arithmetic stays in double.
using Pose3d = std::array<std::array<double, 4>, 4>;

inline Pose3d toPose(const std::vector<std::vector<float>>& transform) {
    Pose3d pose{};
    for(int row = 0; row < 4; ++row) {
        for(int col = 0; col < 4; ++col) pose[row][col] = transform[row][col];
    }
    return pose;
}

inline std::vector<std::vector<float>> fromPose(const Pose3d& pose) {
    std::vector<std::vector<float>> transform(4, std::vector<float>(4, 0.0f));
    for(int row = 0; row < 4; ++row) {
        for(int col = 0; col < 4; ++col) transform[row][col] = static_cast<float>(pose[row][col]);
    }
    return transform;
}

inline Rotation3d rotationOf(const Pose3d& pose) {
    Rotation3d rotation{};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) rotation[row][col] = pose[row][col];
    }
    return rotation;
}

inline Pose3d multiply(const Pose3d& left, const Pose3d& right) {
    Pose3d result{};
    for(int row = 0; row < 4; ++row) {
        for(int col = 0; col < 4; ++col) {
            for(int k = 0; k < 4; ++k) result[row][col] += left[row][k] * right[k][col];
        }
    }
    return result;
}

inline Pose3d inverted(const Pose3d& pose) {
    Pose3d result{};
    result[3][3] = 1.0;
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            result[row][col] = pose[col][row];
            result[row][3] -= pose[col][row] * pose[col][3];
        }
    }
    return result;
}

// Camera center in base coordinates: the base-to-camera transform sends it to the camera frame origin.
inline std::array<double, 3> cameraCenterInBase(const Pose3d& baseToCamera) {
    std::array<double, 3> center{};
    for(int axis = 0; axis < 3; ++axis) {
        for(int row = 0; row < 3; ++row) center[axis] -= baseToCamera[row][axis] * baseToCamera[row][3];
    }
    return center;
}

// Rotation vector of a rotation (axis * angle), and its inverse.
inline std::array<double, 3> rotationLog(const Rotation3d& rotation) {
    std::array<double, 3> skew = {rotation[2][1] - rotation[1][2], rotation[0][2] - rotation[2][0], rotation[1][0] - rotation[0][1]};
    const double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
    const double angle = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
    const double sine = std::sqrt(skew[0] * skew[0] + skew[1] * skew[1] + skew[2] * skew[2]) / 2.0;
    // Near zero the skew part is already twice the rotation vector; near pi it carries no direction, and the
    // averaging iteration is what walks such a turn down to a well conditioned one.
    const double scale = sine < 1e-12 ? 0.5 : angle / (2.0 * sine);
    for(auto& component : skew) component *= scale;
    return skew;
}

inline Rotation3d rotationExp(const std::array<double, 3>& vector) {
    const double angle = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    Rotation3d rotation = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    if(angle < 1e-15) return rotation;

    const std::array<double, 3> unit = {vector[0] / angle, vector[1] / angle, vector[2] / angle};
    const double cross[3][3] = {{0.0, -unit[2], unit[1]}, {unit[2], 0.0, -unit[0]}, {-unit[1], unit[0], 0.0}};
    const double sine = std::sin(angle);
    const double versine = 1.0 - std::cos(angle);
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            double squared = 0.0;
            for(int k = 0; k < 3; ++k) squared += cross[row][k] * cross[k][col];
            rotation[row][col] += sine * cross[row][col] + versine * squared;
        }
    }
    return rotation;
}

// Geodesic mean of a set of rotations: repeatedly average them in the tangent space at the current estimate
// and step there, which converges on the rotation sitting between them.
inline Rotation3d averageRotation(const std::vector<Rotation3d>& rotations) {
    auto mean = rotations.front();
    for(int iteration = 0; iteration < 32; ++iteration) {
        const auto meanInverse = transposed(mean);
        std::array<double, 3> step{};
        for(const auto& rotation : rotations) {
            const auto delta = rotationLog(multiply(meanInverse, rotation));
            for(int axis = 0; axis < 3; ++axis) step[axis] += delta[axis] / static_cast<double>(rotations.size());
        }
        if(std::sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2]) < 1e-15) break;
        mean = multiply(mean, rotationExp(step));
    }
    return mean;
}

// The common in-place turn DynamicCalibrationLibrary gave the rig this round: per camera, the rotation that
// carries that camera's previous orientation onto its new one, read in base coordinates, averaged over the
// calibrated cameras.
inline Rotation3d commonCameraRotation(const CalibrationHandler& current,
                                       const std::variant<CameraBoardSocket, HousingCoordinateSystem>& base,
                                       const std::map<CameraBoardSocket, Pose3d>& calibratedPoses) {
    std::vector<Rotation3d> turns;
    turns.reserve(calibratedPoses.size());
    for(const auto& [socket, calibratedPose] : calibratedPoses) {
        turns.push_back(multiply(transposed(rotationOf(toPose(baseToCameraTransform(current, base, socket)))), rotationOf(calibratedPose)));
    }
    return averageRotation(turns);
}

// Turn every calibrated camera in place - same rotation, each about its own center - so the rig keeps the
// orientation it had while its measured shape stays exactly as the calibration found it. Turning the cameras
// rather than moving the whole rig is what makes this bite: a rigid move of the rig cancels out of every
// camera-to-camera extrinsic, whereas this leaves the relative rotation untouched and re-points the baseline,
// which is the direction the rectified frame is built on.
inline void rotateCamerasInPlace(std::map<CameraBoardSocket, Pose3d>& poses, const Rotation3d& rotation) {
    const auto inverseRotation = transposed(rotation);
    for(auto& [socket, pose] : poses) {
        const auto center = cameraCenterInBase(pose);
        const auto rotated = multiply(rotationOf(pose), inverseRotation);
        for(int row = 0; row < 3; ++row) {
            pose[row][3] = 0.0;
            for(int col = 0; col < 3; ++col) {
                pose[row][col] = rotated[row][col];
                pose[row][3] -= rotated[row][col] * center[col];
            }
        }
    }
}

// All input poses transform the common base (housing, when available) into a camera.
// Every unobserved camera follows the first stereo pair's default depth reference camera through its factory transform.
inline CalibrationHandler assembleDynamicCalibration(const CalibrationHandler& current,
                                                     const CalibrationHandler& factory,
                                                     const std::vector<CameraBoardSocket>& sockets,
                                                     const std::map<CameraBoardSocket, std::vector<std::vector<float>>>& calibratedPoses,
                                                     const std::variant<CameraBoardSocket, HousingCoordinateSystem>& base,
                                                     const std::vector<StereoPair>& stereoPairs,
                                                     Platform platform) {
    if(calibratedPoses.empty()) throw std::invalid_argument("DynamicCalibration has no calibrated camera poses.");

    std::map<CameraBoardSocket, Pose3d> poses;
    for(const auto& [socket, calibratedPose] : calibratedPoses) poses.emplace(socket, toPose(calibratedPose));
    // Undo the common turn before anything derives from these poses, so unobserved cameras follow the corrected
    // rig. The turn is measured against the factory calibration, never against the calibration being replaced:
    // anchoring to the previous cycle would make this an integrator, and correcting a rig that DCL then
    // re-measures feeds back on itself until it runs away.
    const auto& orientationReference = factory.getEepromData().cameraData.empty() ? current : factory;
    rotateCamerasInPlace(poses, commonCameraRotation(orientationReference, base, poses));

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
            poses[socket] = multiply(toPose(factory.getCameraExtrinsics(anchor, socket, false, LengthUnit::METER)), poses.at(anchor));
        }
    }

    auto result = current;
    for(size_t idx = 0; idx + 1 < sockets.size(); ++idx) {
        const auto cameraToNext = multiply(poses.at(sockets[idx + 1]), inverted(poses.at(sockets[idx])));
        auto translation = matrix::extractTranslationVector(fromPose(cameraToNext));
        for(auto& value : translation) value *= 100.0f;
        result.updateCameraExtrinsics(sockets[idx], sockets[idx + 1], matrix::extractRotationMatrix(fromPose(cameraToNext)), translation);
    }

    if(std::holds_alternative<HousingCoordinateSystem>(base)) {
        const auto housingToOrigin = fromPose(poses.at(current.getEepromData().housingExtrinsics.toCameraSocket));
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
