#include "depthai/pipeline/node/SpatialDetectionCalculator.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

SpatialDetectionCalculator::SpatialDetectionCalculator(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, SpatialDetectionCalculator, SpatialDetectionCalculatorProperties>(std::move(props)) {}

SpatialDetectionCalculator& SpatialDetectionCalculator::setDepthThresholds(uint32_t lowerThreshold, uint32_t upperThreshold) {
    properties.lowerThreshold = lowerThreshold;
    properties.upperThreshold = upperThreshold;
    return *this;
}

SpatialDetectionCalculator& SpatialDetectionCalculator::setCalculationAlgorithm(SpatialDetectionCalculatorAlgorithm calculationAlgorithm) {
    properties.calculationAlgorithm = calculationAlgorithm;
    return *this;
}

SpatialDetectionCalculator& SpatialDetectionCalculator::setMeasurementModel(SpatialDetectionsMeasurementModes measurementModel) {
    properties.measurementModel = measurementModel;
    return *this;
}
SpatialDetectionCalculator& SpatialDetectionCalculator::setStepSize(int32_t stepSize) {
    properties.stepSize = stepSize;
    return *this;
}

}  // namespace node
}  // namespace dai
