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

        DEPTHAI_SERIALIZE(AlgorithmControl, recalibrationMode, maximumRecalibrationFrames, maximumCalibCheckFrames);
    };



    std::optional<AlgorithmControl> algorithmControl;

    std::optional<CalibrationCommand> calibrationCommand;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationConfig, algorithmControl, calibrationCommand);
};

}  // namespace dai
