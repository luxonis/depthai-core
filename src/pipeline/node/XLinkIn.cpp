#include "depthai/pipeline/node/XLinkIn.hpp"

namespace dai {
namespace node {

void XLinkIn::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void XLinkIn::setMaxDataSize(std::uint32_t maxDataSize) {
    properties.maxDataSize = maxDataSize;
}

void XLinkIn::setNumFrames(std::uint32_t numFrames) {
    properties.numFrames = numFrames;
}

std::string XLinkIn::getStreamName() const {
    return properties.streamName;
}

std::uint32_t XLinkIn::getMaxDataSize() const {
    return properties.maxDataSize;
}

std::uint32_t XLinkIn::getNumFrames() const {
    return properties.numFrames;
}

}  // namespace node
}  // namespace dai
