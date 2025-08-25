#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include "depthai/pipeline/datatype/Buffer.hpp"
#include <unordered_map>
#include <vector>

namespace dai {

/**
 * DynamicCalibrationConfig message.
 */

struct DynamicCalibrationConfig : public Buffer {
    DynamicCalibrationConfig() = default;
    virtual ~DynamicCalibrationConfig() = default;
    /**
     * Recalibration operating mode.
     *
     * - DEFAULT:
     *   In this mode, recalibration, calibration quality check, and applying the
     *   resulting calibration require explicit user interaction with the pipeline.
     *   The system will not automatically proceed through these steps — the user
     *   must issue commands (e.g., RecalibrateCommand, CalibrationQualityCommand,
     *   ApplyCalibrationCommand) at the appropriate times.
     *
     * - CONTINUOUS:
     *   In this mode, the system automatically handles the full recalibration cycle
     *   without user intervention. Image loading, recalibration, quality checking,
     *   and applying calibration are performed internally in a loop, ensuring the
     *   calibration is kept up to date in real time.
     */
    enum class RecalibrationMode : int32_t { DEFAULT, CONTINUOUS };

    RecalibrationMode recalibrationMode = RecalibrationMode::DEFAULT;

    /**
     * Set the time frequency of recalibration being triggered in Continious mode
     */
    float loadImagePeriod = 0.5;
    float calibrationPeriod = 5;

    /**
     * Performance modes for dynamic calibration.
     *
     * - DEFAULT (0):  
     *   Balanced mode. Uses standard calibration checks and parameters,
     *   suitable for most general use cases.
     *
     * - STATIC_SCENERY (1):  
     *   Optimized for scenarios where the environment and camera remain
     *   fixed (e.g., factory setups, static rigs). Enables stricter checks
     *   to ensure calibration stability over time.
     *
     * - OPTIMIZE_SPEED (2):  
     *   Prioritizes calibration speed over accuracy. Suitable when quick
     *   recalibration is needed, or when calibration must run frequently
     *   with limited compute resources.
     *
     * - OPTIMIZE_PERFORMANCE (3):  
     *   Prioritizes calibration accuracy and robustness, even at the cost
     *   of additional compute time. Suitable for high-precision use cases
     *   where depth quality is critical.
     *
     * - SKIP_CHECKS (4):  
     *   Skips validation and quality checks entirely. Intended for advanced
     *   use cases, debugging, or when external systems handle quality
     *   assurance. Use with caution.
     */
    using PerformanceMode = dcl::PerformanceMode;

    PerformanceMode performanceMode = PerformanceMode::DEFAULT;

    DEPTHAI_SERIALIZE_EXT(DynamicCalibrationConfig, recalibrationMode, performanceMode, loadImagePeriod, calibrationPeriod);
};

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
 * Command to trigger a recalibration step.
 * Can optionally force recalibration regardless of current quality
 * and specify the performance mode to be used.
 */
struct RecalibrateCommand : public DynamicCalibrationCommand {
    RecalibrateCommand(bool force = false, DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT)
        // init in the same order as declared below
        : performanceMode(performanceMode), force(force) {}

    /// Performance mode in which recalibration will run
    DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT;
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
    CalibrationQualityCommand(bool force = false,
                              DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT)
        // match declaration order
        : performanceMode(performanceMode), force(force) {}
    /**
     * If true, bypasses all internal validation checks and triggers calibration quality check immediately.
     * This can speed up capture process, but comes at the cost of reduced accuracy and
     * potentially unstable results. Use with caution.
     */
    DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT;
    bool force = false;
};

/**
 * Command to start capturing for recalibration process.
 * The recalibration will be run in the specified performance mode until stopped.
 */
struct StartRecalibrationCommand : public DynamicCalibrationCommand {
    explicit StartRecalibrationCommand(DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT)
        : performanceMode(performanceMode) {}

    DynamicCalibrationConfig::PerformanceMode performanceMode = DynamicCalibrationConfig::PerformanceMode::DEFAULT;
};

/**
 * Command to stop any ongoing recalibration.
 */
struct StopRecalibrationCommand : public DynamicCalibrationCommand {};

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

}  // namespace dai