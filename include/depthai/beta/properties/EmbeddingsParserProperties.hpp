#pragma once

#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for EmbeddingsParser.
 */
struct EmbeddingsParserProperties : PropertiesSerializable<Properties, EmbeddingsParserProperties> {
    std::string outputLayerName;

    ~EmbeddingsParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(EmbeddingsParserProperties, outputLayerName);

}  // namespace beta
}  // namespace dai
