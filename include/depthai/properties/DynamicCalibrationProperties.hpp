#pragma once
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */
struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    DynamicCalibrationConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, initialConfig);

}  // namespace dai
