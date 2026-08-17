#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "depthai/beta/datatype/FastSAMParserConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for FastSAMParser.
 */
struct FastSAMParserProperties : PropertiesSerializable<Properties, FastSAMParserProperties> {
    FastSAMParserConfig initialConfig;
    std::int32_t numClasses = 1;
    std::vector<std::string> yoloOutputs = {"output1_yolov8", "output2_yolov8", "output3_yolov8"};
    std::vector<std::string> maskOutputs = {"output1_masks", "output2_masks", "output3_masks"};
    std::string protosOutput = "protos_output";

    ~FastSAMParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(FastSAMParserProperties, initialConfig, numClasses, yoloOutputs, maskOutputs, protosOutput);

}  // namespace beta
}  // namespace dai
