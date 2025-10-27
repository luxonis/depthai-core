#pragma once

#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// StereoRectification structure
struct StereoRectification {
    std::vector<std::vector<float>> rectifiedRotationLeft, rectifiedRotationRight;
    CameraBoardSocket leftCameraSocket = CameraBoardSocket::AUTO, rightCameraSocket = CameraBoardSocket::AUTO;
};

DEPTHAI_SERIALIZE_EXT(StereoRectification, rectifiedRotationLeft, rectifiedRotationRight, leftCameraSocket, rightCameraSocket);

}  // namespace dai
