#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

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

#if defined(__clang__)
    ~SPIOutProperties() override;
#else
    virtual ~SPIOutProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(SPIOutProperties, streamName, busId);

}  // namespace dai
