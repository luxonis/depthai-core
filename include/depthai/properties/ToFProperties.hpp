#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ToF
 */
struct ToFProperties : PropertiesSerializable<Properties, ToFProperties> {
    /**
     * Initial ToF config
     */
    ToFConfig initialConfig;

    /**
     * Num frames in output pool
     */
    int numFramesPool = 4;

    /**
     * Number of shaves reserved for ToF decoding.
     */
    std::int32_t numShaves = 1;

    /// Warp HW IDs to use for undistortion, if empty, use auto/default
    std::vector<int> warpHwIds;

    /**
     * Which socket will color camera use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Which camera name will color camera use
     */
    std::string cameraName = "";

    /**
     * Camera sensor image orientation / pixel readout
     */
    CameraImageOrientation imageOrientation = CameraImageOrientation::AUTO;

    /**
     * Camera sensor FPS
     */
    float fps = 30.0;

    /**
     * Pool sizes
     */
    int numFramesPoolRaw = 3;
};

DEPTHAI_SERIALIZE_EXT(ToFProperties, initialConfig, numFramesPool, numShaves, warpHwIds, boardSocket, cameraName, imageOrientation, fps, numFramesPoolRaw);

}  // namespace dai
