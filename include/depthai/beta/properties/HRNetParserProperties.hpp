#pragma once

#include <string>
#include <vector>

#include "depthai/common/KeypointsListT.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for HRNetParser.
 */
struct HRNetParserProperties : PropertiesSerializable<Properties, HRNetParserProperties> {
    std::string outputLayerName;
    float scoreThreshold = 0.5f;
    std::vector<std::string> labelNames;
    std::vector<Edge> edges;

    ~HRNetParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(HRNetParserProperties, outputLayerName, scoreThreshold, labelNames, edges);

}  // namespace beta
}  // namespace dai
