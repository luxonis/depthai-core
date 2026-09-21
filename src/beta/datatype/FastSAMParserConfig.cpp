#include "depthai/beta/datatype/FastSAMParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

FastSAMParserConfig::~FastSAMParserConfig() = default;

void FastSAMParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool FastSAMParserConfig::validate() const {
    const auto isProbability = [](float value) { return value >= 0.0f && value <= 1.0f; };
    if(!isProbability(confidenceThreshold) || !isProbability(iouThreshold) || !isProbability(maskConfidence)) return false;
    if(pointLabel.has_value() && *pointLabel != 0 && *pointLabel != 1) return false;
    if(boundingBox.has_value()) {
        const auto& box = *boundingBox;
        if(box[2] <= 0 || box[3] <= 0 || box[0] >= box[2] || box[1] >= box[3]) {
            return false;
        }
    }
    if(prompt == Prompt::POINT) return points.has_value() && pointLabel.has_value();
    if(prompt == Prompt::BOUNDING_BOX) return boundingBox.has_value();
    return prompt == Prompt::EVERYTHING;
}

void FastSAMParserConfig::setConfidenceThreshold(float threshold) {
    auto candidate = *this;
    candidate.confidenceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Invalid FastSAM parser configuration.");
    confidenceThreshold = threshold;
}

float FastSAMParserConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

void FastSAMParserConfig::setIouThreshold(float threshold) {
    auto candidate = *this;
    candidate.iouThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Invalid FastSAM parser configuration.");
    iouThreshold = threshold;
}

float FastSAMParserConfig::getIouThreshold() const {
    return iouThreshold;
}

void FastSAMParserConfig::setMaskConfidence(float threshold) {
    auto candidate = *this;
    candidate.maskConfidence = threshold;
    DAI_CHECK(candidate.validate(), "Invalid FastSAM parser configuration.");
    maskConfidence = threshold;
}

float FastSAMParserConfig::getMaskConfidence() const {
    return maskConfidence;
}

void FastSAMParserConfig::setPrompt(Prompt value) {
    auto candidate = *this;
    candidate.prompt = value;
    DAI_CHECK(candidate.validate(), "The selected FastSAM prompt is missing its required data.");
    prompt = value;
}

FastSAMParserConfig::Prompt FastSAMParserConfig::getPrompt() const {
    return prompt;
}

void FastSAMParserConfig::setPoints(std::int32_t x, std::int32_t y) {
    points = std::make_pair(x, y);
}

std::optional<std::pair<std::int32_t, std::int32_t>> FastSAMParserConfig::getPoints() const {
    return points;
}

void FastSAMParserConfig::setPointLabel(std::int32_t label) {
    auto candidate = *this;
    candidate.pointLabel = label;
    DAI_CHECK(candidate.validate(), "FastSAM point label must be 0 or 1.");
    pointLabel = label;
}

std::optional<std::int32_t> FastSAMParserConfig::getPointLabel() const {
    return pointLabel;
}

void FastSAMParserConfig::setBoundingBox(const std::array<std::int32_t, 4>& value) {
    auto candidate = *this;
    candidate.boundingBox = value;
    DAI_CHECK(candidate.validate(), "FastSAM bounding box must have ordered coordinates and positive x2/y2 values.");
    boundingBox = value;
}

std::optional<std::array<std::int32_t, 4>> FastSAMParserConfig::getBoundingBox() const {
    return boundingBox;
}

}  // namespace beta
}  // namespace dai
