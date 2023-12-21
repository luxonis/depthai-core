#pragma once

#include <vector>

#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {

/**
 * Specify properties for SpatialLocationCalculator
 */
struct SpatialLocationCalculatorProperties : PropertiesSerializable<Properties, SpatialLocationCalculatorProperties> {
    SpatialLocationCalculatorConfig roiConfig;
};

DEPTHAI_SERIALIZE_EXT(SpatialLocationCalculatorProperties, roiConfig);

}  // namespace dai
