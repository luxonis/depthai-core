#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * MessageDemux does not have any properties to set
 */
struct MessageDemuxProperties : PropertiesSerializable<Properties, MessageDemuxProperties> {
    // Needed for serialization
    char dummy = 0;
};

DEPTHAI_SERIALIZE_EXT(MessageDemuxProperties, dummy);

}  // namespace dai