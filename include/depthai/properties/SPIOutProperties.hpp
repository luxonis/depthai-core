#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * Specify properties for SPIOut node
 */
struct SPIOutProperties : PropertiesSerializable<Properties, SPIOutProperties> {
    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * SPI bus to use
     */
    int busId = 0;
};

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(SPIOutProperties, streamName, busId);

}  // namespace dai
