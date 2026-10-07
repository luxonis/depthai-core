#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/** Configuration for RVC4 visual-inertial odometry. */
struct VIOProperties : PropertiesSerializable<Properties, VIOProperties> {
    /** Accelerometer/gyroscope sample frequency in Hz. */
    int imuFrequency = 200;
    /** Use nominal camera-to-IMU translation from device calibration. */
    bool useSpecTranslation = true;
    ~VIOProperties() override;
};

DEPTHAI_SERIALIZE_EXT(VIOProperties, imuFrequency, useSpecTranslation);

}  // namespace dai
