#pragma once
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */
struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    enum class CalibrationCommand : int32_t {
        START_CALIBRATION_QUALITY_CHECK = 0,        ///< Start calibration quality check
        START_RECALIBRATION = 1,                    ///< Start recalibration
        START_FORCE_CALIBRATION_QUALITY_CHECK = 2,  ///< Start recalibration
        START_FORCE_RECALIBRATION = 3,              ///< Start recalibration
        // Add other datatypes as needed
    };

    enum class PerformanceMode : int32_t { SKIP_CHECKS = 4, STATIC_SCENERY = 1, OPTIMIZE_SPEED = 2, OPTIMIZE_PERFORMANCE = 3, DEFAULT = 0 };

    enum class RecalibrationMode : int32_t { DEFAULT, CONTINUOUS };

    RecalibrationMode recalibrationMode = RecalibrationMode::DEFAULT;
    /**
     * Set the time frequency of recalibration being triggered in Continious mode
     */
    PerformanceMode performanceMode = PerformanceMode::DEFAULT;
    /**
     * Define a peformance mode on which the dynamic recalibration will be working
     */
    uint8_t timeFrequency = 5;

    std::optional<CalibrationCommand> calibrationCommand;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, recalibrationMode, performanceMode, timeFrequency, calibrationCommand);

}  // namespace dai
