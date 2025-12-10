
#pragma once
#include "depthai/properties/Properties.hpp"

namespace dai {

struct DynamicCalibrationWorkerProperties : PropertiesSerializable<Properties, DynamicCalibrationWorkerProperties> {
    enum Mode : int {
        ON_START = 1,
        CONTINUOUS = 2,
    };

    Mode mode = Mode::ON_START;

    int sleepingTime = 600;  // 10 minutes

    ~DynamicCalibrationWorkerProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationWorkerProperties, mode, sleepingTime);

}  // namespace dai
