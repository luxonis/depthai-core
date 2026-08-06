#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "depthai/beta/datatype/RFDETRParserConfig.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for RFDETRParser.
 */
struct RFDETRParserProperties : PropertiesSerializable<Properties, RFDETRParserProperties> {
    RFDETRParserConfig initialConfig;
    std::vector<std::string> labelNames;
    std::vector<std::string> outputLayerNames;
    std::optional<std::pair<std::uint32_t, std::uint32_t>> inputSize;

    ~RFDETRParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RFDETRParserProperties, initialConfig, labelNames, outputLayerNames, inputSize);

}  // namespace beta
}  // namespace dai
