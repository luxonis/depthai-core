#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/beta/datatype/SuperAnimalParserConfig.hpp"
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for SuperAnimalParser.
 */
struct SuperAnimalParserProperties : PropertiesSerializable<Properties, SuperAnimalParserProperties> {
    std::string outputLayerName;
    float scaleFactor = 256.0f;
    std::int64_t nKeypoints = 39;
    SuperAnimalParserConfig initialConfig;
    std::vector<std::string> labelNames;
    std::vector<Edge> edges;

    ~SuperAnimalParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(SuperAnimalParserProperties, outputLayerName, scaleFactor, nKeypoints, initialConfig, labelNames, edges);

}  // namespace beta
}  // namespace dai
