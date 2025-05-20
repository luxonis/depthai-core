#include "depthai/pipeline/node/internal/XLinkOut.hpp"

namespace dai {
namespace node {
namespace internal {
void XLinkOut::buildInternal() {
    // set some default properties
    properties.maxFpsLimit = -1;
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

}  // namespace internal
}  // namespace node
}  // namespace dai
