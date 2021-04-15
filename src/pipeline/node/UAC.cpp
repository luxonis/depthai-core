#include "depthai/pipeline/node/UAC.hpp"

namespace dai {
namespace node {

UAC::UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string UAC::getName() const {
    return "UAC";
}

std::vector<Node::Input> UAC::getInputs() {
    return {};
}

std::vector<Node::Output> UAC::getOutputs() {
    return {};
}

nlohmann::json UAC::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> UAC::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void UAC::setStreamBackMic(bool enable) {
    properties.streamBackMic = enable;
}

}  // namespace node
}  // namespace dai
