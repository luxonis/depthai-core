#include "depthai/pipeline/node/XLinkIn.hpp"

namespace dai {
namespace node {

XLinkIn::XLinkIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NodeCRTP<DeviceNode, XLinkIn, XLinkInProperties>(par, nodeId, std::make_unique<XLinkIn::Properties>()) {
    setOutputRefs(&out);
}

XLinkIn::XLinkIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, XLinkIn, XLinkInProperties>(par, nodeId, std::move(props)) {
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
