#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for SpatialLocationCalculator
 */
struct SpatialLocationCalculatorProperties : PropertiesSerializable<Properties, SpatialLocationCalculatorProperties> {
    SpatialLocationCalculatorConfig roiConfig;

    ~SpatialLocationCalculatorProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(SpatialLocationCalculatorProperties, roiConfig);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::SpatialLocationCalculatorProperties>;
