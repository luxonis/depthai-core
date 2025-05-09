#pragma once
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */
struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    float placeholder = 0.0f;  // Properties not used yet
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, placeholder);

}  // namespace dai
