#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"

#include <opencv2/core/hal/interface.h>

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

}  // namespace dai
