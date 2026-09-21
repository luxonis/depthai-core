#include "depthai/beta/datatype/YuNetParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

YuNetParserConfig::~YuNetParserConfig() = default;

void YuNetParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool YuNetParserConfig::validate() const {
    return confidenceThreshold >= 0.0f && confidenceThreshold <= 1.0f && iouThreshold >= 0.0f && iouThreshold <= 1.0f;
}

void YuNetParserConfig::setConfidenceThreshold(float threshold) {
    auto candidate = *this;
    candidate.confidenceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Confidence threshold must be between 0 and 1.");
    confidenceThreshold = threshold;
}

float YuNetParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void YuNetParserConfig::setIouThreshold(float threshold) {
    auto candidate = *this;
    candidate.iouThreshold = threshold;
    DAI_CHECK(candidate.validate(), "IoU threshold must be between 0 and 1.");
    iouThreshold = threshold;
}

float YuNetParserConfig::getIouThreshold() const {
    return iouThreshold;
}

void YuNetParserConfig::setMaxDetections(int value) {
    auto candidate = *this;
    candidate.maxDetections = value;
    DAI_CHECK(candidate.validate(), "Invalid YuNet parser configuration.");
    maxDetections = value;
}

int YuNetParserConfig::getMaxDetections() const {
    return maxDetections;
}

}  // namespace beta
}  // namespace dai
