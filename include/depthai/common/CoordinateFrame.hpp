#pragma once

#include <string>
#include <tuple>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Identity of a coordinate frame in a (possibly multi-device) setup.
 *
 * A frame is the optical center of one camera on one device. `deviceId` is the device MXID; an empty `deviceId` means
 * *unknown* - it is not a wildcard and is never considered equal to a known device id.
 */
struct CoordinateFrame {
    CoordinateFrame() = default;
    CoordinateFrame(std::string deviceId, CameraBoardSocket socket) : deviceId(std::move(deviceId)), socket(socket) {}
    explicit CoordinateFrame(CameraBoardSocket socket) : socket(socket) {}

    /// Device MXID owning the camera. Empty means unknown / not reported (legacy).
    std::string deviceId;

    /// Camera socket on that device.
    CameraBoardSocket socket = CameraBoardSocket::AUTO;

    /// A frame is fully qualified when both the device and the socket are known.
    bool isQualified() const {
        return !deviceId.empty() && socket != CameraBoardSocket::AUTO;
    }

    /// True if the frame carries no information at all.
    bool isUnknown() const {
        return deviceId.empty() && socket == CameraBoardSocket::AUTO;
    }

    bool operator==(const CoordinateFrame& other) const {
        return deviceId == other.deviceId && socket == other.socket;
    }

    bool operator!=(const CoordinateFrame& other) const {
        return !(*this == other);
    }

    /// Deterministic ordering, used to elect component roots.
    bool operator<(const CoordinateFrame& other) const {
        return std::tie(deviceId, socket) < std::tie(other.deviceId, other.socket);
    }

    DEPTHAI_SERIALIZE(CoordinateFrame, deviceId, socket);
};

inline std::string toString(const CoordinateFrame& frame) {
    return (frame.deviceId.empty() ? std::string("<unknown device>") : frame.deviceId) + ":" + toString(frame.socket);
}

}  // namespace dai

inline std::ostream& operator<<(std::ostream& out, const dai::CoordinateFrame& frame) {
    return out << dai::toString(frame);
}
