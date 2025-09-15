#pragma once
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    bool emptyBool;

    ~DynamicCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, emptyBool);

}  // namespace dai
