#include "depthai/pipeline/node/XLinkOut.hpp"

namespace dai {
namespace node {

XLinkOut::XLinkOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    properties.maxFpsLimit = -1;

    inputs = {&input};
}

std::string XLinkOut::getName() const {
    return "XLinkOut";
}

nlohmann::json XLinkOut::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
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
