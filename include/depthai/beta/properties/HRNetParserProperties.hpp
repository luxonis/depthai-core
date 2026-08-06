#pragma once

#include <string>
#include <vector>

#include "depthai/beta/datatype/HRNetParserConfig.hpp"
#include "depthai/common/KeypointsListT.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for HRNetParser.
 */
struct HRNetParserProperties : PropertiesSerializable<Properties, HRNetParserProperties> {
    std::string outputLayerName;
    HRNetParserConfig initialConfig;
    std::vector<std::string> labelNames;
    std::vector<Edge> edges;

    ~HRNetParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(HRNetParserProperties, outputLayerName, initialConfig, labelNames, edges);

}  // namespace beta
}  // namespace dai
