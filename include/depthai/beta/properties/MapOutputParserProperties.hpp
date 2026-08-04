#pragma once

#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for MapOutputParser.
 */
struct MapOutputParserProperties : PropertiesSerializable<Properties, MapOutputParserProperties> {
    std::string outputLayerName;
    bool minMaxScaling = false;

    ~MapOutputParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(MapOutputParserProperties, outputLayerName, minMaxScaling);

}  // namespace beta
}  // namespace dai
