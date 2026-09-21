#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for LaneDetectionParser.
 */
struct LaneDetectionParserProperties : PropertiesSerializable<Properties, LaneDetectionParserProperties> {
    std::string outputLayerName;
    std::vector<std::int64_t> rowAnchors;
    std::optional<std::int64_t> gridingNum;
    std::optional<std::int64_t> clsNumPerLane;
    std::optional<std::pair<std::uint32_t, std::uint32_t>> inputSize;

    ~LaneDetectionParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(LaneDetectionParserProperties, outputLayerName, rowAnchors, gridingNum, clsNumPerLane, inputSize);

}  // namespace beta
}  // namespace dai
