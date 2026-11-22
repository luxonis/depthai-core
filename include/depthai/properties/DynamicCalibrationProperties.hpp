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

DEPTHAI_SERIALIZE_EXT_DECL(DynamicCalibrationProperties, emptyBool);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::DynamicCalibrationProperties>;
