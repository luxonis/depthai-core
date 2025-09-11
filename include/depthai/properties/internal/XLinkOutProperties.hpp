#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace internal {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

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
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(XLinkOutProperties, maxFpsLimit, streamName, metadataOnly);

}  // namespace internal
}  // namespace dai
