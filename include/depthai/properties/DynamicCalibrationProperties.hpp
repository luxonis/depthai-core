#pragma once
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    bool emptyBool;
};

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, emptyBool);

}  // namespace dai
