#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "depthai/beta/datatype/XFeatMonoParserConfig.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for XFeatMonoParser.
 */
struct XFeatMonoParserProperties : PropertiesSerializable<Properties, XFeatMonoParserProperties> {
    std::string outputLayerFeats = "feats";
    std::string outputLayerKeypoints = "keypoints";
    std::string outputLayerHeatmaps = "heatmaps";
    std::optional<std::pair<std::uint32_t, std::uint32_t>> originalSize;
    std::pair<std::uint32_t, std::uint32_t> inputSize{640, 352};
    XFeatMonoParserConfig initialConfig;

    ~XFeatMonoParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(XFeatMonoParserProperties, outputLayerFeats, outputLayerKeypoints, outputLayerHeatmaps, originalSize, inputSize, initialConfig);

}  // namespace beta
}  // namespace dai
