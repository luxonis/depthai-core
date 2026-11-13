#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "depthai/beta/datatype/MLSDParserConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for MLSDParser.
 */
struct MLSDParserProperties : PropertiesSerializable<Properties, MLSDParserProperties> {
    std::string outputLayerTPMap;
    std::string outputLayerHeat;
    MLSDParserConfig initialConfig;
    std::pair<std::uint32_t, std::uint32_t> inputSize{512, 512};

    ~MLSDParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(MLSDParserProperties, outputLayerTPMap, outputLayerHeat, initialConfig, inputSize);

}  // namespace beta
}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::beta::MLSDParserProperties>;
