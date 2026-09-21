#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/beta/datatype/ClassificationSequenceParserConfig.hpp"
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
    ClassificationSequenceParserConfig initialConfig;

    ~ClassificationSequenceParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ClassificationSequenceParserProperties, outputLayerName, classes, nClasses, isSoftmax, initialConfig);

}  // namespace beta
}  // namespace dai
