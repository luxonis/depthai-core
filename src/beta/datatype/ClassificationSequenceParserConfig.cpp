#include "depthai/beta/datatype/ClassificationSequenceParserConfig.hpp"

#include <algorithm>

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

ClassificationSequenceParserConfig::~ClassificationSequenceParserConfig() = default;

void ClassificationSequenceParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool ClassificationSequenceParserConfig::validate() const {
    return std::all_of(ignoredIndexes.begin(), ignoredIndexes.end(), [](std::int32_t index) { return index >= 0; });
}

void ClassificationSequenceParserConfig::setIgnoredIndexes(const std::vector<std::int32_t>& indexes) {
    auto candidate = *this;
    candidate.ignoredIndexes = indexes;
    DAI_CHECK(candidate.validate(), "Ignored indexes must be nonnegative.");
    ignoredIndexes = indexes;
}

std::vector<std::int32_t> ClassificationSequenceParserConfig::getIgnoredIndexes() const {
    return ignoredIndexes;
}

void ClassificationSequenceParserConfig::setRemoveDuplicates(bool enabled) {
    removeDuplicates = enabled;
}

bool ClassificationSequenceParserConfig::getRemoveDuplicates() const {
    return removeDuplicates;
}

void ClassificationSequenceParserConfig::setConcatenateClasses(bool enabled) {
    concatenateClasses = enabled;
}

bool ClassificationSequenceParserConfig::getConcatenateClasses() const {
    return concatenateClasses;
}

}  // namespace beta
}  // namespace dai
