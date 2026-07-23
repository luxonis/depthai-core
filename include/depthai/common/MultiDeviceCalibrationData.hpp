#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/common/HousingCoordinateSystem.hpp"
#include "depthai/common/Point3f.hpp"

namespace dai {

/**
 * Discriminator for a local frame used by multi-device calibration queries.
 */
enum class MultiDeviceFrameType : uint8_t {
    CAMERA_SOCKET = 0,
    HOUSING = 1,
};

/**
 * Identifies a local frame on a single device.
 *
 * Exactly one payload is active:
 * - `CAMERA_SOCKET`: `cameraSocket != AUTO` and `housing == AUTO`
 * - `HOUSING`: `housing != AUTO` and `cameraSocket == AUTO`
 */
struct MultiDeviceFrame {
    MultiDeviceFrameType type = MultiDeviceFrameType::CAMERA_SOCKET;
    CameraBoardSocket cameraSocket = CameraBoardSocket::AUTO;
    HousingCoordinateSystem housing = HousingCoordinateSystem::AUTO;

    static MultiDeviceFrame camera(CameraBoardSocket socket) {
        MultiDeviceFrame frame;
        frame.type = MultiDeviceFrameType::CAMERA_SOCKET;
        frame.cameraSocket = socket;
        frame.housing = HousingCoordinateSystem::AUTO;
        return frame;
    }

    static MultiDeviceFrame housingFrame(HousingCoordinateSystem housingCoordinateSystem) {
        MultiDeviceFrame frame;
        frame.type = MultiDeviceFrameType::HOUSING;
        frame.cameraSocket = CameraBoardSocket::AUTO;
        frame.housing = housingCoordinateSystem;
        return frame;
    }

    bool operator==(const MultiDeviceFrame& other) const {
        return type == other.type && cameraSocket == other.cameraSocket && housing == other.housing;
    }

    bool operator!=(const MultiDeviceFrame& other) const {
        return !(*this == other);
    }

    DEPTHAI_SERIALIZE(MultiDeviceFrame, type, cameraSocket, housing);
};

/**
 * Rigid transform with a rotation matrix and translation vector.
 */
struct RigidTransform {
    std::array<std::array<float, 3>, 3> rotationMatrix{
        std::array<float, 3>{1.0f, 0.0f, 0.0f},
        std::array<float, 3>{0.0f, 1.0f, 0.0f},
        std::array<float, 3>{0.0f, 0.0f, 1.0f},
    };

    Point3f translation{};
    LengthUnit translationUnit = LengthUnit::CENTIMETER;

    DEPTHAI_SERIALIZE(RigidTransform, rotationMatrix, translation, translationUnit);
};

/**
 * Multi-device calibration entry for one physical device in a rig.
 */
struct MultiDeviceCalibrationDevice {
    std::string mxid;

    /**
     * Full per-device calibration snapshot supplying intrinsics, distortion,
     * camera-to-camera extrinsics, and housing extrinsics.
     */
    EepromData calibration;

    /**
     * Local frame whose pose in the rig is stored in `rigFromAnchor`.
     */
    MultiDeviceFrame anchorFrame;

    /**
     * Homogeneous transform direction: T_rig_from_anchor.
     */
    RigidTransform rigFromAnchor;

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationDevice, mxid, calibration, anchorFrame, rigFromAnchor);
};

/**
 * Serializable multi-device calibration payload.
 */
struct MultiDeviceCalibrationData {
    uint32_t version = 1;

    /**
     * Unix time in milliseconds. Zero means unspecified.
     */
    uint64_t timestamp = 0;

    std::vector<MultiDeviceCalibrationDevice> devices;

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationData, version, timestamp, devices);
};

}  // namespace dai
