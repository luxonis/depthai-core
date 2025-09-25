#include "depthai/pipeline/node/SpatialKeypointsCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialKeypointsCalculator::SpatialKeypointsCalculator(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, SpatialKeypointsCalculator, SpatialKeypointsCalculatorProperties>(std::move(props)) {}

SpatialKeypointsCalculator& SpatialKeypointsCalculator::setDepthThresholds(uint32_t lowerThreshold, uint32_t upperThreshold) {
    properties.lowerThreshold = lowerThreshold;
    properties.upperThreshold = upperThreshold;
    return *this;
}

SpatialKeypointsCalculator& SpatialKeypointsCalculator::setCalculationAlgorithm(SpatialKeypointsCalculatorAlgorithm calculationAlgorithm) {
    properties.calculationAlgorithm = calculationAlgorithm;
    return *this;
}

SpatialKeypointsCalculator& SpatialKeypointsCalculator::setMeasurementModel(SpatialKeypointsMeasurementModes measurementModel) {
    properties.measurementModel = measurementModel;
    return *this;
}
SpatialKeypointsCalculator& SpatialKeypointsCalculator::setStepSize(int32_t stepSize) {
    properties.stepSize = stepSize;
    return *this;
}

}  // namespace node
}  // namespace dai
