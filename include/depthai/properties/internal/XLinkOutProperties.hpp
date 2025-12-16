#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace internal {

/**
 * Specify properties for XLinkOut such as stream name, ...
 */
struct XLinkOutProperties : PropertiesSerializable<Properties, XLinkOutProperties> {
    /**
     * Set a limit to how many packets will be sent further to host
     */
    float maxFpsLimit = -1;

    /**
     * Name of stream
     */
    std::string streamName;
    /**
     * Whether to transfer data or only object attributes
     */
    bool metadataOnly = false;

    /**
     * Max bytes per packet (-1 = unlimited)
     */
    int packetSize = -1;

    /**
     * Maximal bytes per second (-1 = unlimited)
     */
    int bytesPerSecondLimit = -1;

    ~XLinkOutProperties() override;
};

DEPTHAI_SERIALIZE_EXT(XLinkOutProperties, maxFpsLimit, streamName, metadataOnly, packetSize, bytesPerSecondLimit);

}  // namespace internal
}  // namespace dai
