#pragma once

#include "depthai/properties/Properties.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

namespace dai {
namespace internal {

/**
 * Specify properties for XLinkIn such as stream name, ...
 */
struct XLinkInProperties : PropertiesSerializable<Properties, XLinkInProperties> {
    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * Maximum input data size:
     * NV12 --> 1.5 = (3/2) bytes per pixel, roughly 17.6MB for a 4032x3056 frame
     */
    std::uint32_t maxDataSize = ((4032 * 3056) * 3) / 2;

    /**
     * Number of frames in pool
     */
    std::uint32_t numFrames = 8;

    ~XLinkInProperties() override;
};

DEPTHAI_SERIALIZE_EXT(XLinkInProperties, streamName, maxDataSize, numFrames);

}  // namespace internal
}  // namespace dai
