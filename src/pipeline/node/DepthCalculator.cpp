#include "depthai/pipeline/node/DepthCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

DepthCalculator::DepthCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {

}

std::string DepthCalculator::getName() const {
    return "DepthCalculator";
}

std::vector<Node::Output> DepthCalculator::getOutputs() {
    return {out};
}

std::vector<Node::Input> DepthCalculator::getInputs() {
    return {input, depthInput};
}

nlohmann::json DepthCalculator::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

void DepthCalculator::setROIs(std::vector<DepthCalculatorConfig> rois) {
    properties.roiConfig = rois;
}

void DepthCalculator::addROI(DepthCalculatorConfig &roi) {
    properties.roiConfig.push_back(roi);
}


std::shared_ptr<Node> DepthCalculator::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai
