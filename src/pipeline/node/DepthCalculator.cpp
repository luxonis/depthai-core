#include "depthai/pipeline/node/DepthCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

DepthCalculator::DepthCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawDepthCalculatorConfig>()), initialConfig(rawConfig) {}

std::string DepthCalculator::getName() const {
    return "DepthCalculator";
}

std::vector<Node::Output> DepthCalculator::getOutputs() {
    return {out};
}

std::vector<Node::Input> DepthCalculator::getInputs() {
    return {inputConfig, inputDepth};
}

nlohmann::json DepthCalculator::getProperties() {
    nlohmann::json j;
    properties.roiConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
}

// Node properties configuration
void DepthCalculator::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

std::shared_ptr<Node> DepthCalculator::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai
