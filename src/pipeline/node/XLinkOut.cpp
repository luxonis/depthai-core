#include "depthai/pipeline/node/XLinkOut.hpp"

namespace dai {
namespace node {

XLinkOut::XLinkOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    properties.maxFpsLimit = -1;
}

std::string XLinkOut::getName() {
    return "XLinkOut";
}

std::vector<Node::Input> XLinkOut::getInputs() {
    return {input};
}

std::vector<Node::Output> XLinkOut::getOutputs() {
    return {};
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

}  // namespace node
}  // namespace dai
