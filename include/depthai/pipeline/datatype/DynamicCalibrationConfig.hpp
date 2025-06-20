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

        /**
         * Set the running mode of recalibration
         */
        RecalibrationMode recalibrationMode = RecalibrationMode::DEFAULT;

        uint8_t maximumCalibCheckFrames = 5;

        uint8_t maximumRecalibrationFrames = 10;
        /**
         * Enable the coverage Check to be run before the calibration check and recalibration.
         * For better calibration dataset and 2D distribution
         */
        bool enableCoverageCheck = true;

        DEPTHAI_SERIALIZE(AlgorithmControl, recalibrationMode, enableCoverageCheck, maximumRecalibrationFrames, maximumCalibCheckFrames);
    };

    struct CoverageCheckThresholds {
        /**
         * Set the threshold for validation of coverage check, how much of the percentage of the image must be covered
         * Valid range is [0,100]
         */
        uint8_t coverageCheckThreshold = 50;

        DEPTHAI_SERIALIZE(CoverageCheckThresholds, coverageCheckThreshold);
    };

    struct CalibCheckThresholds {
        /**
         * Set threshold for maximum difference in epipolar error before and after the recalibration.
         * Value is set as float, with significant change being marked at around 0.1px
         */
        float epipolarErrorChangeThresholds = 0.1f;

        /**
         * Set threshold for maximum difference in rotation angles before and after recalibration.
         * Value is set as float, with significant change being marked at around 0.1deg
         */
        struct RotationChangeThresholds {
            float x = 0.075;
            float y = 0.1;
            float z = 0.1;

            DEPTHAI_SERIALIZE(RotationChangeThresholds, x, y, z);
        };

        RotationChangeThresholds rotationChangeThresholds;

        DEPTHAI_SERIALIZE(CalibCheckThresholds, epipolarErrorChangeThresholds, rotationChangeThresholds);
    };

    struct RecalibrationThresholds {
        /**
         * Set the pipeline so that device automatically reflash the user calibration
         * after the new calibration is set as device->setCalibration()
         */
        bool flashNewCalibration = false;

        DEPTHAI_SERIALIZE(RecalibrationThresholds, flashNewCalibration);
    };


    std::optional<AlgorithmControl> algorithmControl;

    std::optional<CoverageCheckThresholds> coverageCheckThresholds;

    std::optional<CalibCheckThresholds> calibCheckThresholds;

    std::optional<RecalibrationThresholds> recalibrationThresholds;

    std::optional<CalibrationCommand> calibrationCommand;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationConfig, algorithmControl, coverageCheckThresholds, calibCheckThresholds, recalibrationThresholds, calibrationCommand);
};

}  // namespace dai
