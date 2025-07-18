#pragma once
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    // enum class PerformanceMode : int32_t { SKIP_CHECKS = 4, STATIC_SCENERY = 1, OPTIMIZE_SPEED = 2, OPTIMIZE_PERFORMANCE = 3, DEFAULT = 0 };

    enum class RecalibrationMode : int32_t { DEFAULT, CONTINUOUS };

    RecalibrationMode recalibrationMode = RecalibrationMode::DEFAULT;
    /**
     * Set the time frequency of recalibration being triggered in Continious mode
     */

    using PerformanceMode = dcl::PerformanceMode;

    PerformanceMode performanceMode = PerformanceMode::DEFAULT;
    /**
     * Define a peformance mode on which the dynamic recalibration will be working
     */
    uint8_t loadImageFrequency = 0.5;
    uint8_t calibrationFrequency = 5;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, recalibrationMode, performanceMode, loadImageFrequency);

}  // namespace dai
