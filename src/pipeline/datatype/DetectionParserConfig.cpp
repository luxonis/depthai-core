#include "depthai/pipeline/datatype/DetectionParserConfig.hpp"

namespace dai {

DetectionParserConfig::~DetectionParserConfig() = default;

void DetectionParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::DetectionParserConfig;
}

void DetectionParserConfig::setConfidenceThreshold(const float threshold) {
    this->confidenceThreshold = threshold;
}

float DetectionParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void DetectionParserConfig::setIouThreshold(float threshold) {
    this->iouThreshold = threshold;
}

float DetectionParserConfig::getIouThreshold() const {
    return iouThreshold;
}

}  // namespace dai
