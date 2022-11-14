#include "depthai/pipeline/node/XLinkIn.hpp"

namespace dai {
namespace node {

XLinkIn::XLinkIn() : NodeCRTP<DeviceNode, XLinkIn, XLinkInProperties>(std::make_unique<XLinkIn::Properties>()) {
    setOutputRefs(&out);
}

XLinkIn::XLinkIn(std::unique_ptr<Properties> props) : NodeCRTP<DeviceNode, XLinkIn, XLinkInProperties>(std::move(props)) {
    setOutputRefs(&out);
}

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
