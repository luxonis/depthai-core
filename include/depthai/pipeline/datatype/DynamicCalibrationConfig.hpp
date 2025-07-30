#pragma once

#include <DynamicCalibration.hpp>
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
    DynamicCalibrationConfig() = default;
    virtual ~DynamicCalibrationConfig() = default;

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
    float loadImageFrequency = 0.5;
    float calibrationFrequency = 5;

    DEPTHAI_SERIALIZE_EXT(DynamicCalibrationConfig, recalibrationMode, performanceMode, loadImageFrequency);
};

struct DynamicCalibrationCommand : public Buffer {
    enum class Command : int32_t {
        START_CALIBRATION_QUALITY_CHECK = 0,        ///< Start calibration quality check
        START_RECALIBRATION = 1,                    ///< Start recalibration
        START_FORCE_CALIBRATION_QUALITY_CHECK = 2,  ///< Start recalibration
        START_FORCE_RECALIBRATION = 3,              ///< Start recalibration
        START_LOADING_IMAGES = 4,
        STOP_LOADING_IMAGES = 5,
        APPLY_NEW_CALIBRATION = 6,
        APPLY_PREVIOUS_CALIBRATION = 7,
        APPLY_INITIAL_CALIBRATION = 8,
    };

    Command calibrationCommand;

    DynamicCalibrationCommand() = default;
    virtual ~DynamicCalibrationCommand() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };

    DEPTHAI_SERIALIZE_EXT(DynamicCalibrationCommand, calibrationCommand);
};

}  // namespace dai
