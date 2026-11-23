
#pragma once
#include "depthai/pipeline/datatype/AutoCalibrationConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct AutoCalibrationProperties : PropertiesSerializable<Properties, AutoCalibrationProperties> {
    AutoCalibrationConfig initialConfig;

    /// Rate in frames per second at which the gate lets synced frames through to the calibration.
    int gateFps = 5;

    ~AutoCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(AutoCalibrationProperties, initialConfig, gateFps);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::AutoCalibrationProperties>;
