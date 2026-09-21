#include "depthai/beta/datatype/MPPalmDetectionParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

MPPalmDetectionParserConfig::~MPPalmDetectionParserConfig() = default;

void MPPalmDetectionParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool MPPalmDetectionParserConfig::validate() const {
    return confidenceThreshold >= 0.0f && confidenceThreshold <= 1.0f && iouThreshold >= 0.0f && iouThreshold <= 1.0f && maxDetections > 0;
}

void MPPalmDetectionParserConfig::setConfidenceThreshold(float threshold) {
    auto candidate = *this;
    candidate.confidenceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Confidence threshold must be between 0 and 1.");
    confidenceThreshold = threshold;
}

float MPPalmDetectionParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void MPPalmDetectionParserConfig::setIouThreshold(float threshold) {
    auto candidate = *this;
    candidate.iouThreshold = threshold;
    DAI_CHECK(candidate.validate(), "IoU threshold must be between 0 and 1.");
    iouThreshold = threshold;
}

float MPPalmDetectionParserConfig::getIouThreshold() const {
    return iouThreshold;
}

void MPPalmDetectionParserConfig::setMaxDetections(int value) {
    auto candidate = *this;
    candidate.maxDetections = value;
    DAI_CHECK(candidate.validate(), "Maximum detections must be positive.");
    maxDetections = value;
}

int MPPalmDetectionParserConfig::getMaxDetections() const {
    return maxDetections;
}

}  // namespace beta
}  // namespace dai
