#include "depthai/pipeline/node/XLinkOut.hpp"

namespace dai {
namespace node {

XLinkOut::XLinkOut() : NodeCRTP<DeviceNode, XLinkOut, XLinkOutProperties>(std::make_unique<XLinkOut::Properties>()) {
    properties.maxFpsLimit = -1;
    setInputRefs(&input);
}

XLinkOut::XLinkOut(std::unique_ptr<Properties> props) : NodeCRTP<DeviceNode, XLinkOut, XLinkOutProperties>(std::move(props)) {
    setInputRefs(&input);
}

void XLinkOut::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void XLinkOut::setFpsLimit(float fps) {
    properties.maxFpsLimit = fps;
}

void XLinkOut::setMetadataOnly(bool metadataOnly) {
    properties.metadataOnly = metadataOnly;
}

std::string XLinkOut::getStreamName() const {
    return properties.streamName;
}

float XLinkOut::getFpsLimit() const {
    return properties.maxFpsLimit;
}

bool XLinkOut::getMetadataOnly() const {
    return properties.metadataOnly;
}

}  // namespace node
}  // namespace dai
