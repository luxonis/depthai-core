#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
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
    float loadImagePeriod = 0.5;
    float calibrationPeriod = 5;

    DEPTHAI_SERIALIZE_EXT(DynamicCalibrationConfig, recalibrationMode, performanceMode, loadImagePeriod, calibrationPeriod);
};

struct DynamicCalibrationCommand : public Buffer {
    DynamicCalibrationCommand() = default;
    virtual ~DynamicCalibrationCommand() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationCommand;
    };

    // DEPTHAI_SERIALIZE_EXT(DynamicCalibrationCommand)
};

struct RecalibrateCommand : public DynamicCalibrationCommand {
    RecalibrateCommand(bool force = false, DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT)
        // init in the same order as declared below
        : performanceMode(performanceMode), force(force) {}

    DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT;
    bool force = false;
};

struct CalibrationQualityCommand : public DynamicCalibrationCommand {
    CalibrationQualityCommand(bool force = false,
                              DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT)
        // match declaration order
        : performanceMode(performanceMode), force(force) {}

    DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT;
    bool force = false;
};

struct StartRecalibrationCommand : public DynamicCalibrationCommand {
    explicit StartRecalibrationCommand(DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT)
        : performanceMode(performanceMode) {}

    DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT;
};

struct StopRecalibrationCommand : public DynamicCalibrationCommand {};

struct LoadImageCommand : public DynamicCalibrationCommand {};

struct ApplyCalibrationCommand : public DynamicCalibrationCommand {
    ApplyCalibrationCommand() = default;
    explicit ApplyCalibrationCommand(const CalibrationHandler& calibration) : calibration(calibration) {}
    CalibrationHandler calibration;
};

}  // namespace dai
