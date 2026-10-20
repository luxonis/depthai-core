#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for ClassificationParser.
 */
struct ClassificationParserProperties : PropertiesSerializable<Properties, ClassificationParserProperties> {
    std::string outputLayerName;
    std::vector<std::string> classes;
    std::int64_t nClasses = 0;
    bool isSoftmax = true;

    ~ClassificationParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ClassificationParserProperties, outputLayerName, classes, nClasses, isSoftmax);

}  // namespace beta
}  // namespace dai
