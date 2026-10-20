#include "depthai/beta/datatype/PPTextDetectionParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

PPTextDetectionParserConfig::~PPTextDetectionParserConfig() = default;

void PPTextDetectionParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool PPTextDetectionParserConfig::validate() const {
    return confidenceThreshold >= 0.0f && confidenceThreshold <= 1.0f && maskThreshold >= 0.0f && maskThreshold <= 1.0f && maxDetections > 0;
}

void PPTextDetectionParserConfig::setConfidenceThreshold(float threshold) {
    auto candidate = *this;
    candidate.confidenceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Confidence threshold must be between 0 and 1.");
    confidenceThreshold = threshold;
}

float PPTextDetectionParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void PPTextDetectionParserConfig::setMaskThreshold(float threshold) {
    auto candidate = *this;
    candidate.maskThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Mask threshold must be between 0 and 1.");
    maskThreshold = threshold;
}

float PPTextDetectionParserConfig::getMaskThreshold() const {
    return maskThreshold;
}

void PPTextDetectionParserConfig::setMaxDetections(int value) {
    auto candidate = *this;
    candidate.maxDetections = value;
    DAI_CHECK(candidate.validate(), "Maximum detections must be positive.");
    maxDetections = value;
}

int PPTextDetectionParserConfig::getMaxDetections() const {
    return maxDetections;
}

}  // namespace beta
}  // namespace dai
