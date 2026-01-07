#pragma once

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for SpatialDetectionNetwork
 */
struct SpatialDetectionNetworkProperties : PropertiesSerializable<Properties, SpatialDetectionNetworkProperties> {
    SpatialLocationCalculatorConfigThresholds depthThresholds;
    SpatialLocationCalculatorAlgorithm calculationAlgorithm = SpatialLocationCalculatorAlgorithm::MEDIAN;
    std::int32_t stepSize = SpatialLocationCalculatorConfigData::AUTO;

    ~SpatialDetectionNetworkProperties() override;
};

DEPTHAI_SERIALIZE_EXT(SpatialDetectionNetworkProperties, depthThresholds, calculationAlgorithm, stepSize);

}  // namespace dai
