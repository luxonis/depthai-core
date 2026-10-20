#pragma once

#include <string>

#include "depthai/beta/datatype/MapOutputParserConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for MapOutputParser.
 */
struct MapOutputParserProperties : PropertiesSerializable<Properties, MapOutputParserProperties> {
    std::string outputLayerName;
    MapOutputParserConfig initialConfig;

    ~MapOutputParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(MapOutputParserProperties, outputLayerName, initialConfig);

}  // namespace beta
}  // namespace dai
