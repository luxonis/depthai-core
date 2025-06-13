#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialLocationCalculator::SpatialLocationCalculator(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties>(std::move(props)) {}

SpatialLocationCalculator::Properties& SpatialLocationCalculator::getProperties() {
    properties.roiConfig = *initialConfig;
    return properties;
}

}  // namespace node
}  // namespace dai
