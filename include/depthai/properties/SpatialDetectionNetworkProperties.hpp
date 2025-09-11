#pragma once

// std
#include <vector>

// libraries

// project
#include "depthai/common/DetectionNetworkType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/**
 * Specify properties for SpatialDetectionNetwork
 */
struct SpatialDetectionNetworkProperties : PropertiesSerializable<Properties, SpatialDetectionNetworkProperties> {
    float detectedBBScaleFactor = 1.0;
    SpatialLocationCalculatorConfigThresholds depthThresholds;
    SpatialLocationCalculatorAlgorithm calculationAlgorithm = SpatialLocationCalculatorAlgorithm::MEDIAN;
    std::int32_t stepSize = SpatialLocationCalculatorConfigData::AUTO;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(SpatialDetectionNetworkProperties, detectedBBScaleFactor, depthThresholds, calculationAlgorithm);

}  // namespace dai
