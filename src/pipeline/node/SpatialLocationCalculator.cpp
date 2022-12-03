#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialLocationCalculator::SpatialLocationCalculator()
    : NodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties>(),
      rawConfig(std::make_shared<RawSpatialLocationCalculatorConfig>()),
      initialConfig(rawConfig) {}

SpatialLocationCalculator::SpatialLocationCalculator(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties>(std::move(props)),
      rawConfig(std::make_shared<RawSpatialLocationCalculatorConfig>(properties.roiConfig)),
      initialConfig(rawConfig) {}

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
