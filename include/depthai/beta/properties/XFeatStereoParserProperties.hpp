#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for XFeatStereoParser.
 */
struct XFeatStereoParserProperties : PropertiesSerializable<Properties, XFeatStereoParserProperties> {
    std::string outputLayerFeats = "feats";
    std::string outputLayerKeypoints = "keypoints";
    std::string outputLayerHeatmaps = "heatmaps";
    std::optional<std::pair<std::uint32_t, std::uint32_t>> originalSize;
    std::pair<std::uint32_t, std::uint32_t> inputSize{640, 352};
    int maxKeypoints = 4096;

    ~XFeatStereoParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(XFeatStereoParserProperties, outputLayerFeats, outputLayerKeypoints, outputLayerHeatmaps, originalSize, inputSize, maxKeypoints);

}  // namespace beta
}  // namespace dai
