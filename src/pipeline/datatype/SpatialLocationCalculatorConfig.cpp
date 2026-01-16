#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {

SpatialLocationCalculatorConfig::~SpatialLocationCalculatorConfig() = default;

void SpatialLocationCalculatorConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SpatialLocationCalculatorConfig;
}

void SpatialLocationCalculatorConfig::setROIs(std::vector<SpatialLocationCalculatorConfigData> ROIs) {
    config = ROIs;
}

void SpatialLocationCalculatorConfig::addROI(SpatialLocationCalculatorConfigData& ROI) {
    config.push_back(ROI);
}

std::vector<SpatialLocationCalculatorConfigData> SpatialLocationCalculatorConfig::getConfigData() const {
    return config;
}

void SpatialLocationCalculatorConfig::setDepthThresholds(uint32_t lowerThreshold, uint32_t upperThreshold) {
    if(lowerThreshold >= upperThreshold) {
        throw std::invalid_argument("Lower threshold must be less than upper threshold");
    }
    globalLowerThreshold = lowerThreshold;
    globalUpperThreshold = upperThreshold;
}

void SpatialLocationCalculatorConfig::setCalculationAlgorithm(SpatialLocationCalculatorAlgorithm calculationAlgorithm) {
    globalCalculationAlgorithm = calculationAlgorithm;
}

void SpatialLocationCalculatorConfig::setStepSize(int32_t stepSize) {
    if(stepSize < -1 || stepSize == 0) {
        throw std::invalid_argument("Step size must be -1 (AUTO) or a positive integer.");
    }
    globalStepSize = stepSize;
}

void SpatialLocationCalculatorConfig::setKeypointRadius(int32_t radius) {
    if(radius <= 0) {
        throw std::invalid_argument("Keypoint radius must be positive");
    }
    globalKeypointRadius = radius;
}

void SpatialLocationCalculatorConfig::setCalculateSpatialKeypoints(bool calculateSpatialKeypoints) {
    this->calculateSpatialKeypoints = calculateSpatialKeypoints;
}

void SpatialLocationCalculatorConfig::setUseSegmentation(bool useSegmentation) {
    this->useSegmentation = useSegmentation;
}

void SpatialLocationCalculatorConfig::setSegmentationPassthrough(bool passthroughSegmentation) {
    this->segmentationPassthrough = passthroughSegmentation;
}

std::pair<int32_t, int32_t> SpatialLocationCalculatorConfig::getDepthThresholds() const {
    return {globalLowerThreshold, globalUpperThreshold};
}

SpatialLocationCalculatorAlgorithm SpatialLocationCalculatorConfig::getCalculationAlgorithm() const {
    return globalCalculationAlgorithm;
}

int32_t SpatialLocationCalculatorConfig::getStepSize() const {
    return globalStepSize;
}

int32_t SpatialLocationCalculatorConfig::getKeypointRadius() const {
    return globalKeypointRadius;
}

bool SpatialLocationCalculatorConfig::getCalculateSpatialKeypoints() const {
    return calculateSpatialKeypoints;
}

bool SpatialLocationCalculatorConfig::getUseSegmentation() const {
    return useSegmentation;
}

bool SpatialLocationCalculatorConfig::getSegmentationPassthrough() const {
    return segmentationPassthrough;
}

}  // namespace dai
