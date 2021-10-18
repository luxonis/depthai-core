#include "depthai/pipeline/node/XLinkOut.hpp"

namespace dai {
namespace node {

XLinkOut::XLinkOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : XLinkOut(par, nodeId, std::make_unique<XLinkOut::Properties>()) {}
XLinkOut::XLinkOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : Node(par, nodeId, std::move(props)), properties(static_cast<Properties&>(*Node::properties)) {
    properties.maxFpsLimit = -1;

    inputs = {&input};
}

std::string XLinkOut::getName() const {
    return "XLinkOut";
}

XLinkOut::Properties& XLinkOut::getProperties() {
    return properties;
}

std::shared_ptr<Node> XLinkOut::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
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
