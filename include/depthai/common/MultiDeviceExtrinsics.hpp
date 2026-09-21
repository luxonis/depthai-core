#pragma once

#include <string>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * A directed cross-device calibration edge.
 *
 * The source coordinate system is identified by fromDeviceId/fromSocket. The
 * destination coordinate system is identified by extrinsics.toDeviceId and
 * extrinsics.toCameraSocket.
 */
struct MultiDeviceExtrinsics {
    std::string fromDeviceId;
    CameraBoardSocket fromSocket = CameraBoardSocket::AUTO;
    Extrinsics extrinsics;
};

DEPTHAI_SERIALIZE_EXT(MultiDeviceExtrinsics, fromDeviceId, fromSocket, extrinsics);

}  // namespace dai
