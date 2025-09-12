#pragma once

#include "depthai/properties/Properties.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

namespace dai {

/**
 * Properties for SPIIn node
 */
struct SPIInProperties : PropertiesSerializable<Properties, SPIInProperties> {
    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * SPI bus to use
     */
    int busId = 0;

    /**
     * Maximum input data size
     */
    std::uint32_t maxDataSize = dai::device::XLINK_USB_BUFFER_MAX_SIZE;

    /**
     * Number of frames in pool
     */
    std::uint32_t numFrames = 4;

#if defined(__clang__)
    ~SPIInProperties() override;
#else
    virtual ~SPIInProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(SPIInProperties, streamName, busId, maxDataSize, numFrames);

}  // namespace dai
