#include "depthai/beta/datatype/HRNetParserConfig.hpp"

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

HRNetParserConfig::~HRNetParserConfig() = default;

void HRNetParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool HRNetParserConfig::validate() const {
    return scoreThreshold >= 0.0f && scoreThreshold <= 1.0f;
}

void HRNetParserConfig::setScoreThreshold(float threshold) {
    auto candidate = *this;
    candidate.scoreThreshold = threshold;
    DAI_CHECK(candidate.validate(), "Score threshold must be between 0 and 1.");
    scoreThreshold = threshold;
}

float HRNetParserConfig::getScoreThreshold() const {
    return scoreThreshold;
}

}  // namespace beta
}  // namespace dai
