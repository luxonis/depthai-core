#include "depthai/pipeline/node/internal/XLinkOut.hpp"

namespace dai {
namespace node {
namespace internal {

void XLinkOut::buildInternal() {
    // set some default properties
    properties.maxFpsLimit = -1;
}
// Setters
void XLinkOut::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void XLinkOut::setMetadataOnly(bool metadataOnly) {
    properties.metadataOnly = metadataOnly;
}

void XLinkOut::setFpsLimit(float fps) {
    properties.maxFpsLimit = fps;
}

void XLinkOut::setPacketSize(int packetSize) {
    properties.packetSize = packetSize;
}

void XLinkOut::setPacketFrequency(int packetFrequency) {
    properties.packetFrequency = packetFrequency;
}
// Getters

std::string XLinkOut::getStreamName() const {
    return properties.streamName;
}

bool XLinkOut::getMetadataOnly() const {
    return properties.metadataOnly;
}

float XLinkOut::getFpsLimit() const {
    return properties.maxFpsLimit;
}

int XLinkOut::getPacketSize() const {
    return properties.packetSize;
}

int XLinkOut::getPacketFrequency() const {
    return properties.packetFrequency;
}

}  // namespace internal
}  // namespace node
}  // namespace dai
