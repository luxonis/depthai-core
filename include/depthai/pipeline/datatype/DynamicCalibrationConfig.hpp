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
        /**
         * Enable the coverage Check to be run before the calibration check and recalibration.
         * For better calibration dataset and 2D distribution
         */
        bool enableCoverageCheck = true;

        DEPTHAI_SERIALIZE(AlgorithmControl,
                          recalibrationMode,
                          periodTime,
                          timeUnit,
                          enableCoverageCheck);
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

    AlgorithmControl algorithmControl;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationConfig, algorithmControl);
};

}  // namespace dai
