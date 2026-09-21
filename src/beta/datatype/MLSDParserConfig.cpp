#include "depthai/beta/datatype/MLSDParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

MLSDParserConfig::~MLSDParserConfig() = default;

void MLSDParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool MLSDParserConfig::validate() const {
    return topK > 0 && scoreThreshold >= 0.0f && scoreThreshold <= 1.0f && distanceThreshold >= 0.0f;
}

void MLSDParserConfig::setTopK(int value) {
    auto candidate = *this;
    candidate.topK = value;
    DAI_CHECK(candidate.validate(), "Top K must be positive.");
    topK = value;
}

int MLSDParserConfig::getTopK() const {
    return topK;
}

void MLSDParserConfig::setScoreThreshold(float threshold) {
    auto candidate = *this;
    candidate.scoreThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Score threshold must be between 0 and 1.");
    scoreThreshold = threshold;
}

float MLSDParserConfig::getScoreThreshold() const {
    return scoreThreshold;
}

void MLSDParserConfig::setDistanceThreshold(float threshold) {
    auto candidate = *this;
    candidate.distanceThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Distance threshold must be nonnegative.");
    distanceThreshold = threshold;
}

float MLSDParserConfig::getDistanceThreshold() const {
    return distanceThreshold;
}

}  // namespace beta
}  // namespace dai
