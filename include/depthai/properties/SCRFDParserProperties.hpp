#pragma once

#include <cstdint>
#include <vector>

#include "build/_deps/xtensor-src/include/xtensor/core/xtensor_forward.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/properties/Properties.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Specify properties for DetectionParser
 */
struct SCRFDParserProperties : PropertiesSerializable<Properties, SCRFDParserProperties> {
    /// Num frames in output pool
    int numFramesPool = 8;

    bool runOnHostVar = false;

    /// Network inputs
    std::unordered_map<std::string, TensorInfo> networkInputs;

    std::vector<std::string> outputLayerNames;
    std::vector<std::string> labelNames = {"Face"};
    float confidenceThreshold = 0.5;
    float IoUThreshold = 0.5;
    int maxDet = 100;
    std::array<int, 2> inputSize = {640, 640};
    std::array<int, 3> featureStrideFPN = {8, 16, 32};
    int numberOfAnchors = 2;
    std::map anchorCenters;
};

DEPTHAI_SERIALIZE_EXT(SCRFDParserProperties, numFramesPool, networkInputs, runOnHostVar);

}  // namespace dai
