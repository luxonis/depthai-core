#pragma once
#include "depthai/properties/Properties.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */
struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {

    DynamicCalibrationConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, initialConfig);

}  // namespace dai
