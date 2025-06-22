#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {


/**
 * DynamicCalibrationConfig message.
 */
struct DynamicCalibrationConfig : public Buffer {

    enum class CalibrationCommand : int32_t {
        START_CALIBRATION_QUALITY_CHECK = 0,  ///< Start calibration quality check
        START_RECALIBRATION = 1,              ///< Start recalibration
        START_FORCE_CALIBRATION_QUALITY_CHECK = 2,              ///< Start recalibration
        START_FORCE_RECALIBRATION = 3,              ///< Start recalibration
        // Add other datatypes as needed
    };

    /**
     * Construct DynamicCalibrationConfig message.
     */
    DynamicCalibrationConfig() = default;
    virtual ~DynamicCalibrationConfig() = default;

    struct AlgorithmControl {
        /**
         * Define a mode in which recalibration will be running
         */
        enum class RecalibrationMode : int32_t { DEFAULT, CONTINUOUS };

        RecalibrationMode recalibrationMode = RecalibrationMode::DEFAULT;
        /**
         * Define a peformance mode on which the dynamic recalibration will be working
         */
        enum class PerformanceMode: int32_t { SKIP_CHECKS = 4, STATIC_SCENERY = 1, OPTIMIZE_SPEED = 2, OPTIMIZE_PEFRORMACE = 3, DEFAULT = 0};
        
        PerformanceMode performanceMode =  PerformanceMode::DEFAULT;
        /**
         * Set the time frequency of recalibration being triggered in Continious mode
         */
        uint8_t timeFrequency = 5;

        DEPTHAI_SERIALIZE(AlgorithmControl, recalibrationMode, performanceMode, timeFrequency);
    };



    AlgorithmControl algorithmControl;

    std::optional<CalibrationCommand> calibrationCommand;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationConfig, algorithmControl, calibrationCommand);
};

}  // namespace dai
