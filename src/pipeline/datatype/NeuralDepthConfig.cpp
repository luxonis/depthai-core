#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"

namespace dai {

NeuralDepthConfig::~NeuralDepthConfig() = default;

void NeuralDepthConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::NeuralDepthConfig;
}

uint8_t NeuralDepthConfig::getConfidenceThreshold() const {
    return postProcessing.confidenceThreshold;
}

NeuralDepthConfig& NeuralDepthConfig::setConfidenceThreshold(uint8_t confThr) {
    postProcessing.confidenceThreshold = confThr;
    return *this;
}

uint8_t NeuralDepthConfig::getEdgeThreshold() const {
    return postProcessing.edgeThreshold;
}

NeuralDepthConfig& NeuralDepthConfig::setEdgeThreshold(uint8_t edgeThr) {
    postProcessing.edgeThreshold = edgeThr;
    return *this;
}

NeuralDepthConfig& NeuralDepthConfig::setDepthUnit(AlgorithmControl::DepthUnit depthUnit) {
    algorithmControl.depthUnit = depthUnit;
    return *this;
}

NeuralDepthConfig::AlgorithmControl::DepthUnit NeuralDepthConfig::getDepthUnit() const {
    return algorithmControl.depthUnit;
}

NeuralDepthConfig& NeuralDepthConfig::setCustomDepthUnitMultiplier(float multiplier) {
    algorithmControl.customDepthUnitMultiplier = multiplier;
    return *this;
}

float NeuralDepthConfig::getCustomDepthUnitMultiplier() const {
    return algorithmControl.customDepthUnitMultiplier;
}

}  // namespace dai
