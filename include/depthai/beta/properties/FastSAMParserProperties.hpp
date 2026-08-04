#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for FastSAMParser.
 */
struct FastSAMParserProperties : PropertiesSerializable<Properties, FastSAMParserProperties> {
    float confidenceThreshold = 0.5f;
    std::int32_t numClasses = 1;
    float iouThreshold = 0.5f;
    float maskConfidence = 0.5f;
    std::string prompt = "everything";
    std::optional<std::pair<std::int32_t, std::int32_t>> points;
    std::optional<std::int32_t> pointLabel;
    std::optional<std::array<std::int32_t, 4>> boundingBox;
    std::vector<std::string> yoloOutputs = {"output1_yolov8", "output2_yolov8", "output3_yolov8"};
    std::vector<std::string> maskOutputs = {"output1_masks", "output2_masks", "output3_masks"};
    std::string protosOutput = "protos_output";

    ~FastSAMParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(FastSAMParserProperties,
                      confidenceThreshold,
                      numClasses,
                      iouThreshold,
                      maskConfidence,
                      prompt,
                      points,
                      pointLabel,
                      boundingBox,
                      yoloOutputs,
                      maskOutputs,
                      protosOutput);

}  // namespace beta
}  // namespace dai
