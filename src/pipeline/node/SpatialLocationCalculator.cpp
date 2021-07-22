#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialLocationCalculator::SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawSpatialLocationCalculatorConfig>()), initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputDepth};
    outputs = {&out, &passthroughDepth};
}

std::string SpatialLocationCalculator::getName() const {
    return "SpatialLocationCalculator";
}

nlohmann::json SpatialLocationCalculator::getProperties() {
    nlohmann::json j;
    properties.roiConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
}

// Node properties configuration
void SpatialLocationCalculator::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

std::shared_ptr<Node> SpatialLocationCalculator::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai
