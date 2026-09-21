#pragma once

#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for RegressionParser.
 */
struct RegressionParserProperties : PropertiesSerializable<Properties, RegressionParserProperties> {
    std::string outputLayerName;

    ~RegressionParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RegressionParserProperties, outputLayerName);

}  // namespace beta
}  // namespace dai
