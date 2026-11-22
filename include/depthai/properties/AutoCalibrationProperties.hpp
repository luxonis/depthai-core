
#pragma once
#include "depthai/pipeline/datatype/AutoCalibrationConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct AutoCalibrationProperties : PropertiesSerializable<Properties, AutoCalibrationProperties> {
    AutoCalibrationConfig initialConfig;

    ~AutoCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(AutoCalibrationProperties, initialConfig);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::AutoCalibrationProperties>;
