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

/**
 * @brief Control message used to manage Dynamic Calibration on the device.
 *
 * This class encapsulates all commands that can be sent to the DynamicCalibration node.
 * A command is represented using the @ref Command variant, which may contain one of the
 * command structures defined in the @ref Commands namespace.
 *
 * Factory methods (e.g., @ref calibrate, @ref startCalibration) allow convenient
 * creation of command objects.
 *
 * The message is serialized and transmitted to the device through an input queue.
 */
class DynamicCalibrationControl : public Buffer {
   public:
    /**
     * @brief Performance mode for Dynamic Calibration.
     *
     * These modes influence how aggressively calibration and quality checks
     * are performed on the device.
     */
    enum class PerformanceMode : int {
        DEFAULT,               ///< Default behavior (balance quality and speed)
        STATIC_SCENERY,        ///< Optimized for scenes with little or no motion
        OPTIMIZE_SPEED,        ///< Favor faster processing at possible quality trade-off
        OPTIMIZE_PERFORMANCE,  ///< Favor accuracy even at reduced speed
        SKIP_CHECKS            ///< Skip quality checks; force calibration actions
    };

    /**
     * @brief Namespace containing all command structures that may be used inside @ref Command.
     *
     * These structs represent individual commands that the Dynamic Calibration node
     * can execute. Each command corresponds to a specific action or configuration update.
     */
    struct Commands {
        /**
         * @brief Command to perform a full calibration run.
         */
        struct Calibrate {
            explicit Calibrate(bool force = false) : force(force) {}
            bool force = false;  ///< Force calibration even when unnecessary.
            DEPTHAI_SERIALIZE(Calibrate, force);
        };

        /**
         * @brief Command to perform a calibration quality check.
         */
        struct CalibrationQuality {
            explicit CalibrationQuality(bool force = false) : force(force) {}
            bool force = false;  ///< Force check even if previous quality is valid.
            DEPTHAI_SERIALIZE(CalibrationQuality, force);
        };

        /**
         * @brief Command to start dynamic calibration with periodic operations.
         *
         * @param loadImagePeriod How often image loading should occur (seconds)
         * @param calibrationPeriod How often calibration should run (seconds)
         */
        struct StartCalibration {
            explicit StartCalibration(float loadImagePeriod = 0.5f, float calibrationPeriod = 5.0f)
                : loadImagePeriod(loadImagePeriod), calibrationPeriod(calibrationPeriod) {}

            float loadImagePeriod = 0.5f;    ///< Seconds between image loads.
            float calibrationPeriod = 5.0f;  ///< Seconds between calibration cycles.

            DEPTHAI_SERIALIZE(StartCalibration, loadImagePeriod, calibrationPeriod);
        };

        /**
         * @brief Command to stop a running dynamic calibration process.
         */
        struct StopCalibration {};

        /**
         * @brief Command to load a single image for calibration.
         *
         * Useful for manual step-by-step calibration sequences.
         */
        struct LoadImage {};

        /**
         * @brief Command to apply a new calibration to the device.
         */
        struct ApplyCalibration {
            ApplyCalibration() = default;
            explicit ApplyCalibration(const CalibrationHandler& calibration) : calibration(calibration) {}

            CalibrationHandler calibration;  ///< Calibration data to apply.
            DEPTHAI_SERIALIZE(ApplyCalibration, calibration);
        };

        /**
         * @brief Command to reset accumulated calibration and quality data.
         */
        struct ResetData {};

        /**
         * @brief Command to select the calibration performance mode.
         */
        struct SetPerformanceMode {
            SetPerformanceMode() : performanceMode(PerformanceMode::DEFAULT) {}
            explicit SetPerformanceMode(PerformanceMode performanceMode) : performanceMode(performanceMode) {}

            PerformanceMode performanceMode;  ///< Desired performance mode.
            DEPTHAI_SERIALIZE(SetPerformanceMode, performanceMode);
        };
    };

    /**
     * @brief Variant type representing a single calibration command.
     *
     * Only one command may be active at any time. If no command is present,
     * the variant contains `std::monostate`.
     */
    using Command = std::variant<std::monostate,                ///< No command
                                 Commands::Calibrate,           ///< Run calibration
                                 Commands::CalibrationQuality,  ///< Run calibration quality check
                                 Commands::StartCalibration,    ///< Start periodic calibration
                                 Commands::StopCalibration,     ///< Stop calibration
                                 Commands::LoadImage,           ///< Load an image
                                 Commands::ApplyCalibration,    ///< Apply calibration data
                                 Commands::ResetData,           ///< Reset calibration data
                                 Commands::SetPerformanceMode   ///< Set performance mode
                                 >;

    Command command{};

    DynamicCalibrationControl() {}

    explicit DynamicCalibrationControl(Command cmd) : command(std::move(cmd)) {}

    ~DynamicCalibrationControl() override;

    // ---------- Named constructors that return shared_ptr ----------
    /**
     * @brief Create a command that triggers a full calibration run.
     *
     * @param force If true, force calibration even if the quality check passes.
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> calibrate(bool force = false) {
        return std::make_shared<DynamicCalibrationControl>(Commands::Calibrate{force});
    }

    /**
     * @brief Create a command to request a calibration quality check.
     *
     * @param force If true, force execution even if conditions are not ideal.
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> calibrationQuality(bool force = false) {
        return std::make_shared<DynamicCalibrationControl>(Commands::CalibrationQuality{force});
    }

    /**
     * @brief Start periodic dynamic calibration.
     *
     * @param loadImagePeriod How often images should be loaded, in seconds.
     * @param calibrationPeriod How often a new calibration should be executed, in seconds.
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> startCalibration(float loadImagePeriod = 0.5f, float calibrationPeriod = 5.0f) {
        return std::make_shared<DynamicCalibrationControl>(Commands::StartCalibration{loadImagePeriod, calibrationPeriod});
    }

    /**
     * @brief Stop a running dynamic calibration session.
     *
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> stopCalibration() {
        return std::make_shared<DynamicCalibrationControl>(Commands::StopCalibration{});
    }

    /**
     * @brief Request manual image loading for calibration.
     *
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> loadImage() {
        return std::make_shared<DynamicCalibrationControl>(Commands::LoadImage{});
    }

    /**
     * @brief Apply a new calibration to the system.
     *
     * @param calibration Calibration data to apply.
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> applyCalibration(const CalibrationHandler& calibration) {
        return std::make_shared<DynamicCalibrationControl>(Commands::ApplyCalibration{calibration});
    }

    /**
     * @brief Reset internal calibration-related data.
     *
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
    [[nodiscard]] static std::shared_ptr<DynamicCalibrationControl> resetData() {
        return std::make_shared<DynamicCalibrationControl>(Commands::ResetData{});
    }

    /**
     * @brief Set the performance mode for dynamic calibration.
     *
     * @param mode Performance mode to use. Defaults to PerformanceMode::DEFAULT.
     * @return Shared pointer to a DynamicCalibrationControl command.
     */
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
