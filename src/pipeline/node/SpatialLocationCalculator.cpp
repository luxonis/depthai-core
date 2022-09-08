#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialLocationCalculator::SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties>(par, nodeId, std::make_unique<SpatialLocationCalculator::Properties>()),
      rawConfig(std::make_shared<RawSpatialLocationCalculatorConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputDepth});
    setOutputRefs({&out, &passthroughDepth});
}

SpatialLocationCalculator::SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawSpatialLocationCalculatorConfig>(properties.roiConfig)),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputDepth});
    setOutputRefs({&out, &passthroughDepth});
}

SpatialLocationCalculator::Properties& SpatialLocationCalculator::getProperties() {
    properties.roiConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void SpatialLocationCalculator::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool SpatialLocationCalculator::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

}  // namespace node
}  // namespace dai
