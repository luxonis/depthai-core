#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * MessageDemux does not have any properties to set
 */
struct MessageDemuxProperties : PropertiesSerializable<Properties, MessageDemuxProperties> {
    // Needed for serialization
    char dummy = 0;
};

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(MessageDemuxProperties, dummy);

}  // namespace dai