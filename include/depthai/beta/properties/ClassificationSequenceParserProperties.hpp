#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for ClassificationSequenceParser.
 */
struct ClassificationSequenceParserProperties : PropertiesSerializable<Properties, ClassificationSequenceParserProperties> {
    std::string outputLayerName;
    std::vector<std::string> classes;
    std::int64_t nClasses = 0;
    bool isSoftmax = true;
    std::vector<std::int32_t> ignoredIndexes;
    bool removeDuplicates = false;
    bool concatenateClasses = false;

    ~ClassificationSequenceParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(
    ClassificationSequenceParserProperties, outputLayerName, classes, nClasses, isSoftmax, ignoredIndexes, removeDuplicates, concatenateClasses);

}  // namespace beta
}  // namespace dai
