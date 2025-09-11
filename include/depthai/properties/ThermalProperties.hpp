#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/pipeline/datatype/ThermalConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * Specify properties for Thermal
 */
struct ThermalProperties : PropertiesSerializable<Properties, ThermalProperties> {
    /**
     * Initial Thermal config
     */
    ThermalConfig initialConfig;

    /**
     * Num frames in output pool
     */
    int numFramesPool = 4;

    /**
     * Which socket will color camera use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Camera sensor FPS
     */
    float fps = 25.0;
};
#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(ThermalProperties, initialConfig, numFramesPool, boardSocket, fps);

}  // namespace dai
