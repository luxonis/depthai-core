#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * SystemLoggerProperties structure
 */
struct SystemLoggerProperties : PropertiesSerializable<Properties, SystemLoggerProperties> {
    /**
     * Rate at which the messages are going to be sent in hertz
     */
    float rateHz = 1.0f;
};

DEPTHAI_SERIALIZE_EXT(SystemLoggerProperties, rateHz);

}  // namespace dai
