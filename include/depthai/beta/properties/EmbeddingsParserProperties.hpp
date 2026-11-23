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

DEPTHAI_SERIALIZE_EXT_DECL(EmbeddingsParserProperties, outputLayerName);

}  // namespace beta
}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::beta::EmbeddingsParserProperties>;
