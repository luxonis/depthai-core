#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialLocationCalculator::SpatialLocationCalculator(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties>(std::move(props)) {}

SpatialLocationCalculator::Properties& SpatialLocationCalculator::getProperties() {
    properties.roiConfig = initialConfig;
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
