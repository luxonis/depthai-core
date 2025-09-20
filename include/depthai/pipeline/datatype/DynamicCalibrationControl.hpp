#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <depthai/common/variant.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/export.hpp"

namespace dai {

class DEPTHAI_API DynamicCalibrationControl : public Buffer {
   public:
    enum class PerformanceMode : int { DEFAULT, STATIC_SCENERY, OPTIMIZE_SPEED, OPTIMIZE_PERFORMANCE, SKIP_CHECKS };

    struct Commands {
        struct Calibrate {
            explicit Calibrate(bool force = false) : force(force) {}
            bool force = false;
            DEPTHAI_SERIALIZE(Calibrate, force);
        };

        struct CalibrationQuality {
            explicit CalibrationQuality(bool force = false) : force(force) {}
            bool force = false;
            DEPTHAI_SERIALIZE(CalibrationQuality, force);
        };

        struct StartCalibration {
            explicit StartCalibration(float loadImagePeriod = 0.5f, float calibrationPeriod = 5.0f)
                : loadImagePeriod(loadImagePeriod), calibrationPeriod(calibrationPeriod) {}
            float loadImagePeriod = 0.5f;
            float calibrationPeriod = 5.0f;
            DEPTHAI_SERIALIZE(StartCalibration, loadImagePeriod, calibrationPeriod);
        };

        struct StopCalibration {};

        struct LoadImage {};

        struct ApplyCalibration {
            ApplyCalibration() = default;
            explicit ApplyCalibration(const CalibrationHandler& calibration) : calibration(calibration) {}
            CalibrationHandler calibration;
            DEPTHAI_SERIALIZE(ApplyCalibration, calibration);
        };

        struct ResetData {};

        struct SetPerformanceMode {
            SetPerformanceMode() : performanceMode(PerformanceMode::DEFAULT) {}  // optional default
            explicit SetPerformanceMode(PerformanceMode performanceMode) : performanceMode(performanceMode) {}
            PerformanceMode performanceMode;
            DEPTHAI_SERIALIZE(SetPerformanceMode, performanceMode);
        };
    };

    using Command = std::variant<std::monostate,  // if no command is present
                                 Commands::Calibrate,
                                 Commands::CalibrationQuality,
                                 Commands::StartCalibration,
                                 Commands::StopCalibration,
                                 Commands::LoadImage,
                                 Commands::ApplyCalibration,
                                 Commands::ResetData,
                                 Commands::SetPerformanceMode>;

    Command command{};

    DynamicCalibrationControl() {}

    explicit DynamicCalibrationControl(Command cmd) : command(std::move(cmd)) {}
    ~DynamicCalibrationControl() override;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationControl;
    }
};

}  // namespace dai
