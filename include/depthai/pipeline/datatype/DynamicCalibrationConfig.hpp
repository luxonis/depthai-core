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
    bool force = false;
};

struct CalibrationQualityCommand : public DynamicCalibrationCommand {
    bool force = false;
};

struct StartRecalibrationCommand : public DynamicCalibrationCommand {};

struct LoadImageCommand : public DynamicCalibrationCommand {};

struct ApplyCalibrationCommand : public DynamicCalibrationCommand {
    ApplyCalibrationCommand() = default;
    ApplyCalibrationCommand(const CalibrationHandler& calibration) : calibration(calibration) {}
    CalibrationHandler calibration;
};

}  // namespace dai
