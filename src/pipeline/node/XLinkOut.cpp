#include "depthai/pipeline/node/XLinkOut.hpp"

namespace dai {
namespace node {

std::shared_ptr<XLinkOut> XLinkOut::build() {
    // set some default properties
    properties.maxFpsLimit = -1;

    isBuild = true;
    return std::static_pointer_cast<XLinkOut>(shared_from_this());
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
