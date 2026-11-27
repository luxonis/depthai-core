#include "depthai/pipeline/node/internal/PeerLinkIn.hpp"

namespace dai {
namespace node {
namespace internal {

void PeerLinkIn::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void PeerLinkIn::setProducerEndpoints(const std::vector<std::string>& endpoints) {
    properties.producerEndpoints = endpoints;
}

std::string PeerLinkIn::getStreamName() const {
    return properties.streamName;
}

std::vector<std::string> PeerLinkIn::getProducerEndpoints() const {
    return properties.producerEndpoints;
}

}  // namespace internal
}  // namespace node
}  // namespace dai
