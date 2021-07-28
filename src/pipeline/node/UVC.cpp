#include "depthai/pipeline/node/UVC.hpp"

namespace dai {
namespace node {

UVC::UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string UVC::getName() const {
    return "UVC";
}

std::vector<Node::Input> UVC::getInputs() {
    return {input};
}

std::vector<Node::Output> UVC::getOutputs() {
    return {};
}

nlohmann::json UVC::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> UVC::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void UVC::setGpiosOnInit(std::unordered_map<int, int> list) {
    properties.gpioInit = list;
}

void UVC::setGpiosOnStreamOn(std::unordered_map<int, int> list) {
    properties.gpioStreamOn = list;
}

void UVC::setGpiosOnStreamOff(std::unordered_map<int, int> list) {
    properties.gpioStreamOff = list;
}

}  // namespace node
}  // namespace dai
