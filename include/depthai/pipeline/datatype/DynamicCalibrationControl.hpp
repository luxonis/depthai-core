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

namespace dai {

class DynamicCalibrationControl : public Buffer {
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

    // ---------- Named constructors that return shared_ptr ----------
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> calibrate(bool force = false) {
        return std::make_shared<DynamicCalibrationControl>(Commands::Calibrate{force});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> calibrationQuality(bool force = false) {
        return std::make_shared<DynamicCalibrationControl>(Commands::CalibrationQuality{force});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> startCalibration(float loadImagePeriod = 0.5f, float calibrationPeriod = 5.0f) {
        return std::make_shared<DynamicCalibrationControl>(Commands::StartCalibration{loadImagePeriod, calibrationPeriod});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> stopCalibration() {
        return std::make_shared<DynamicCalibrationControl>(Commands::StopCalibration{});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> loadImage() {
        return std::make_shared<DynamicCalibrationControl>(Commands::LoadImage{});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> applyCalibration(const CalibrationHandler& calibration) {
        return std::make_shared<DynamicCalibrationControl>(Commands::ApplyCalibration{calibration});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> resetData() {
        return std::make_shared<DynamicCalibrationControl>(Commands::ResetData{});
    }

    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> setPerformanceMode(PerformanceMode mode = PerformanceMode::DEFAULT) {
        return std::make_shared<DynamicCalibrationControl>(Commands::SetPerformanceMode{mode});
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::DynamicCalibrationControl;
    }
};

}  // namespace dai
