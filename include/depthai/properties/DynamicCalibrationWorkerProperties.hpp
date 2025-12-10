
#pragma once
#include "depthai/pipeline/datatype/DynamicCalibrationWorkerConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct DynamicCalibrationWorkerProperties : PropertiesSerializable<Properties, DynamicCalibrationWorkerProperties> {
    DynamicCalibrationWorkerConfig initialConfig;

    ~DynamicCalibrationWorkerProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationWorkerProperties, initialConfig);

}  // namespace dai
