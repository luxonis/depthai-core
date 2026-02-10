#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"

namespace dai {

SegmentationParserConfig::~SegmentationParserConfig() = default;

void SegmentationParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SegmentationParserConfig;
}

void SegmentationParserConfig::setConfidenceThreshold(const float threshold) {
    this->confidenceThreshold = threshold;
}

float SegmentationParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void SegmentationParserConfig::setStepSize(unsigned int stepSize) {
    this->stepSize = stepSize;
}

unsigned int SegmentationParserConfig::getStepSize() const {
    return stepSize;
}

}  // namespace dai
