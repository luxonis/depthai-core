#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <depthai/common/variant.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class DynamicCalibrationControl : public Buffer {
   public:
    // ----- Commands (declare these first) -----

    struct CalibrateCommand {
        explicit CalibrateCommand(bool force = false) : force(force) {}
        bool force = false;
        DEPTHAI_SERIALIZE(CalibrateCommand, force);
    };

    struct CalibrationQualityCommand {
        explicit CalibrationQualityCommand(bool force = false) : force(force) {}
        bool force = false;
        DEPTHAI_SERIALIZE(CalibrationQualityCommand, force);
    };

    struct StartCalibrationCommand {
        explicit StartCalibrationCommand(float loadImagePeriod = 0.5f, float calibrationPeriod = 5.0f)
            : loadImagePeriod(loadImagePeriod), calibrationPeriod(calibrationPeriod) {}
        float loadImagePeriod = 0.5f;
        float calibrationPeriod = 5.0f;
        DEPTHAI_SERIALIZE(StartCalibrationCommand, loadImagePeriod, calibrationPeriod);
    };

    struct StopCalibrationCommand {};

    struct LoadImageCommand {};

    struct ApplyCalibrationCommand {
        ApplyCalibrationCommand() = default;
        explicit ApplyCalibrationCommand(const CalibrationHandler& calibration) : calibration(calibration) {}
        CalibrationHandler calibration;
        DEPTHAI_SERIALIZE(ApplyCalibrationCommand, calibration);
    };

    struct ResetDataCommand {};

    struct SetPerformanceModeCommand {
        SetPerformanceModeCommand() : performanceMode(dcl::PerformanceMode::DEFAULT) {}  // optional default
        explicit SetPerformanceModeCommand(dcl::PerformanceMode performanceMode) : performanceMode(performanceMode) {}
        dcl::PerformanceMode performanceMode;
        DEPTHAI_SERIALIZE(SetPerformanceModeCommand, performanceMode);
    };

    // ----- Variant (after all command types) -----
    using Command = std::variant<CalibrateCommand,
                                 CalibrationQualityCommand,
                                 StartCalibrationCommand,
                                 StopCalibrationCommand,
                                 LoadImageCommand,
                                 ApplyCalibrationCommand,
                                 ResetDataCommand,
                                 SetPerformanceModeCommand>;

    Command command;

    DynamicCalibrationControl() : command(CalibrateCommand{}) {}

    explicit DynamicCalibrationControl(Command cmd) : command(std::move(cmd)) {}

    // Convenience overloads
    explicit DynamicCalibrationControl(const CalibrateCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const CalibrationQualityCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const StartCalibrationCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const StopCalibrationCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const LoadImageCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const ApplyCalibrationCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const ResetDataCommand& c) : command(c) {}
    explicit DynamicCalibrationControl(const SetPerformanceModeCommand& c) : command(c) {}

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationControl;
    }
};

}  // namespace dai
