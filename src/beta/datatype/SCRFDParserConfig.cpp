#include "depthai/beta/datatype/SCRFDParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

SCRFDParserConfig::~SCRFDParserConfig() = default;

void SCRFDParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool SCRFDParserConfig::validate() const {
    return confidenceThreshold >= 0.0f && confidenceThreshold <= 1.0f && iouThreshold >= 0.0f && iouThreshold <= 1.0f && maxDetections > 0;
}

void SCRFDParserConfig::setConfidenceThreshold(float threshold) {
    auto candidate = *this;
    candidate.confidenceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Confidence threshold must be between 0 and 1.");
    confidenceThreshold = threshold;
}

float SCRFDParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void SCRFDParserConfig::setIouThreshold(float threshold) {
    auto candidate = *this;
    candidate.iouThreshold = threshold;
    DAI_CHECK(candidate.validate(), "IoU threshold must be between 0 and 1.");
    iouThreshold = threshold;
}

float SCRFDParserConfig::getIouThreshold() const {
    return iouThreshold;
}

void SCRFDParserConfig::setMaxDetections(int value) {
    auto candidate = *this;
    candidate.maxDetections = value;
    DAI_CHECK(candidate.validate(), "Maximum detections must be positive.");
    maxDetections = value;
}

int SCRFDParserConfig::getMaxDetections() const {
    return maxDetections;
}

}  // namespace beta
}  // namespace dai
