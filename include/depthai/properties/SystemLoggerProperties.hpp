#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraImageOrientation.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

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
#ifdef __clang__
#pragma clang diagnostic pop
#endif

}  // namespace dai
