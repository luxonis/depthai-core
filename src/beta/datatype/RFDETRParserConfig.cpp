#include "depthai/beta/datatype/RFDETRParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

RFDETRParserConfig::~RFDETRParserConfig() = default;

void RFDETRParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool RFDETRParserConfig::validate() const {
    return confidenceThreshold >= 0.0f && confidenceThreshold <= 1.0f && maxDetections > 0 && maskConfidence >= 0.0f && maskConfidence <= 1.0f;
}

void RFDETRParserConfig::setConfidenceThreshold(float threshold) {
    auto candidate = *this;
    candidate.confidenceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Confidence threshold must be between 0 and 1.");
    confidenceThreshold = threshold;
}

float RFDETRParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void RFDETRParserConfig::setMaxDetections(int value) {
    auto candidate = *this;
    candidate.maxDetections = value;
    DAI_CHECK(candidate.validate(), "Maximum detections must be positive.");
    maxDetections = value;
}

int RFDETRParserConfig::getMaxDetections() const {
    return maxDetections;
}

void RFDETRParserConfig::setMaskConfidence(float threshold) {
    auto candidate = *this;
    candidate.maskConfidence = threshold;
    DAI_CHECK(candidate.validate(), "Mask confidence must be between 0 and 1.");
    maskConfidence = threshold;
}

float RFDETRParserConfig::getMaskConfidence() const {
    return maskConfidence;
}

}  // namespace beta
}  // namespace dai
