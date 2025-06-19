#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * StereoDepthConfig message.
 */
class DynamicCalibrationConfig : public Buffer {
   public:
    /**
     * Construct StereoDepthConfig message.
     */
    DynamicCalibrationConfig() = default;
    virtual ~DynamicCalibrationConfig() = default;

    struct AlgorithmControl {
        /**
         * Define a mode in which recalibration will be running
         */
        enum class RecalibrationMode : int32_t { DEFAULT, CONTINIOUS };

        /**
         * Measurement unit for time period before next calibration
         */
        enum class TimeUnit : int32_t { SECONDS, MINUTES, HOURS };

        /**
         * Set the running mode of recalibration
         */
        RecalibrationMode recalibrationMode = RecalibrationMode::DEFAULT;

        /**
         * Control over the DCL with usage of FPS and time
         * All values are int
         */
        TimeUnit timeUnit = TimeUnit::SECONDS;

        uint8_t periodTime = 5; 

        uint8_t maximumCalibCheckFrames = 5;

        uint8_t maximumRecalibrationFrames = 10;
        /**
         * Enable the coverage Check to be run before the calibration check and recalibration.
         * For better calibration dataset and 2D distribution
         */
        bool enableCoverageCheck = true;

        DEPTHAI_SERIALIZE(AlgorithmControl,
                          recalibrationMode,
                          periodTime,
                          timeUnit,
                          enableCoverageCheck,
                          maximumRecalibrationFrames,
                          maximumCalibCheckFrames);
    };

    struct CoverageCheckThresholds {
        /**
         * Set the threshold for validation of coverage check, how much of the percentage of the image must be covered
         * Valid range is [0,100]
         */
        uint8_t coverageCheckThreshold = 50;
        
        DEPTHAI_SERIALIZE(CoverageCheckThresholds,
                          coverageCheckThreshold);
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

        DEPTHAI_SERIALIZE(CalibCheckThresholds,
                          epipolarErrorChangeThresholds,
                          rotationChangeThresholds
                          );
    };


    struct RecalibrationThresholds {
        /**
         * Set the pipeline so that device automatically reflash the user calibration 
         * after the new calibration is set as device->setCalibration()
         */
        bool flashNewCalibration = false;

        DEPTHAI_SERIALIZE(RecalibrationThresholds,
                          flashNewCalibration
                          );
    };

    /**
     * @param mode Set operating mode, CONTINIOUS OR DEFAULT
     */
    DynamicCalibrationConfig& setCalibrationMode(AlgorithmControl::RecalibrationMode mode);

    /**
     * Set the initial time, when should the recalibration proceed
     * @param time INT
     * @param timeUnit SEC, MIN, H
     */
    DynamicCalibrationConfig& setTimePeriod(uint8_t time, AlgorithmControl::TimeUnit timeUnit);
    /**
     * Get time period between processes.
     */
    uint8_t getTimePeriod() const;

    /**
     * @param enable Enable or disable the coverage Check based on the scenery you are using. For example, if we have a static scenery
     * it would be better to set it as False.
     */
    DynamicCalibrationConfig& setEnableCoverageCheck(bool enable);
    /**
     * Get bool for enabled or disabled CoverageCheck
     */
    bool getEnableCoverageCheck() const;

    DynamicCalibrationConfig& setMaximumCalibCheckFrames(uint8_t maxFramesPerCheck);

    uint8_t getMaximumCalibCheckFrames() const;

    DynamicCalibrationConfig& setMaximumRecalibrationFrames(uint8_t maxFramesPerRecalib);

    uint8_t getMaximumRecalibrationFrames() const;

    DynamicCalibrationConfig& setCoverageCheckThreshold(uint8_t coverageThreshold);

    uint8_t getCoverageCheckThreshold() const;

    DynamicCalibrationConfig& setEpipolarErrorChangeThresholds(float value);

    float getEpipolarErrorChangeThresholds() const;

    CalibCheckThresholds::RotationChangeThresholds getRotationChangeThresholds() const;

    DynamicCalibrationConfig& setFlashNewCalibration(bool enable);


    AlgorithmControl algorithmControl;


    CoverageCheckThresholds coverageCheckThresholds;


    CalibCheckThresholds calibCheckThresholds;


    RecalibrationThresholds recalibrationThresholds;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationConfig, algorithmControl, coverageCheckThresholds, calibCheckThresholds, recalibrationThresholds);
};

}  // namespace dai
