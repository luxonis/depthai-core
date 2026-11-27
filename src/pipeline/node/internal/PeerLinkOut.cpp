#include "depthai/pipeline/node/internal/PeerLinkOut.hpp"

namespace dai {
namespace node {
namespace internal {

void PeerLinkOut::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void PeerLinkOut::setFpsLimit(float fps) {
    properties.maxFpsLimit = fps;
}

std::string PeerLinkOut::getStreamName() const {
    return properties.streamName;
}

float PeerLinkOut::getFpsLimit() const {
    return properties.maxFpsLimit;
}

}  // namespace internal
}  // namespace node
}  // namespace dai
