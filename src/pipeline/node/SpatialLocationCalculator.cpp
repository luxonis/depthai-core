#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialLocationCalculator::SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : SpatialLocationCalculator(par, nodeId, std::make_unique<SpatialLocationCalculator::Properties>()) {}
SpatialLocationCalculator::SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : Node(par, nodeId, std::move(props)),
      properties(static_cast<Properties&>(*Node::properties)),
      rawConfig(std::make_shared<RawSpatialLocationCalculatorConfig>()),
      initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputDepth};
    outputs = {&out, &passthroughDepth};
}

std::string SpatialLocationCalculator::getName() const {
    return "SpatialLocationCalculator";
}

SpatialLocationCalculator::Properties& SpatialLocationCalculator::getProperties() {
    properties.roiConfig = *rawConfig;
    return properties;
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
