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
 * Base class for all dynamic calibration control commands.
 */
struct DynamicCalibrationCommand : public Buffer {
    DynamicCalibrationCommand() = default;
    virtual ~DynamicCalibrationCommand() = default;
    /**
     * Serialize the command to metadata and set the datatype.
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationCommand;
    };

    // DEPTHAI_SERIALIZE_EXT(DynamicCalibrationCommand)
};

/**
 * Command to trigger a calibration step.
 * Can optionally force calibration regardless of current quality
 * and specify the performance mode to be used.
 */
struct CalibrateCommand : public DynamicCalibrationCommand {
    CalibrateCommand(bool force = false)
        // init in the same order as declared below
        : force(force) {}

    /**
     * If true, bypasses all internal validation checks and triggers calibration immediately.
     * This can speed up capture process, but comes at the cost of reduced accuracy and
     * potentially unstable results. Use with caution.
     */
    bool force = false;
};

/**
 * Command to check calibration quality.
 * Can optionally force evaluation and specify the performance mode context.
 */
struct CalibrationQualityCommand : public DynamicCalibrationCommand {
    CalibrationQualityCommand(bool force = false)
        // match declaration order
        : force(force) {}
    /**
     * If true, bypasses all internal validation checks and triggers calibration quality check immediately.
     * This can speed up capture process, but comes at the cost of reduced accuracy and
     * potentially unstable results. Use with caution.
     */
    bool force = false;
};

/**
 * Command to start capturing for calibration process.
 * The calibration will be run in the specified performance mode until stopped.
 */
struct StartCalibrationCommand : public DynamicCalibrationCommand {
    explicit StartCalibrationCommand(float loadImagePeriod = 0.5f, float calibrationPeriod = 5.0f)
        : loadImagePeriod(loadImagePeriod), calibrationPeriod(calibrationPeriod) {}

    float loadImagePeriod = 0.5f;
    float calibrationPeriod = 5.0f;
};

/**
 * Command to stop any ongoing calibration.
 */
struct StopCalibrationCommand : public DynamicCalibrationCommand {};

/**
 * Command to load a new image into the calibration process.
 * Typically used to feed external image data instead of capturing from the device.
 */
struct LoadImageCommand : public DynamicCalibrationCommand {};

/**
 * Command to apply a calibration directly to the device.
 */
struct ApplyCalibrationCommand : public DynamicCalibrationCommand {
    ApplyCalibrationCommand() = default;
    /**
     * Construct with a specific calibration to be applied.
     */
    explicit ApplyCalibrationCommand(const CalibrationHandler& calibration) : calibration(calibration) {}

    CalibrationHandler calibration;
};

struct ResetDataCommand : public DynamicCalibrationCommand {};

struct SetPerformanceModeCommand : public DynamicCalibrationCommand {
    explicit SetPerformanceModeCommand(const dcl::PerformanceMode performanceMode) : performanceMode(performanceMode) {}

    const dcl::PerformanceMode performanceMode = dcl::PerformanceMode::DEFAULT;
};

}  // namespace dai
