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

    ~SPIOutProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(SPIOutProperties, streamName, busId);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::SPIOutProperties>;
