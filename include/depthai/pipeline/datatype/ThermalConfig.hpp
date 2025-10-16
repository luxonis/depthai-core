#pragma once
#include <depthai/common/optional.hpp>
#include <optional>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * ThermalConfig message. Currently unused.
 */
class ThermalConfig : public Buffer {
   public:
    /// Orientation of the image.
    enum ThermalImageOrientation { Normal, Mirror, Flip, MirrorFlip };

    /**
     * Thermal sensor gain mode.
     * Use low gain in high energy environments.
     */
    enum ThermalGainMode { LOW, HIGH };

    /**
     * Ambient factors that affect the temperature measurement of a Thermal sensor.
     */
    struct ThermalAmbientParams {
        /// Distance to the measured object. unit:cnt(128cnt=1m), range:0-25600(0-200m)
        std::optional<uint16_t> distance;

        /// Reflection temperature. unit:K, range:230-500(high gain), 230-900(low gain)
        std::optional<uint16_t> reflectionTemperature;

        /// Atmospheric temperature. unit:K, range:230-500(high gain), 230-900(low gain)
        std::optional<uint16_t> atmosphericTemperature;

        /// Emissivity. unit:1/128, range:1-128(0.01-1)
        std::optional<uint8_t> targetEmissivity;

        /// Atmospheric transmittance. unit:1/128, range:1-128(0.01-1)
        std::optional<uint8_t> atmosphericTransmittance;

        /// Gain mode, low or high.
        std::optional<ThermalGainMode> gainMode;
        DEPTHAI_SERIALIZE(ThermalAmbientParams, distance, reflectionTemperature, atmosphericTemperature, targetEmissivity, atmosphericTransmittance, gainMode);
    };

    struct ThermalFFCParams {
        /// Auto Flat-Field-Correction. Controls wheather the shutter is controlled by the sensor module automatically or not.
        std::optional<bool> autoFFC;

        /// Minimum FFC interval when auto FFC is enabled.
        /// The time interval between two FFC should not be less than this value.
        std::optional<uint16_t> minFFCInterval;

        /// Maximum FFC interval when auto FFC is enabled.
        /// The time interval between two FFC should not be more than this value.
        std::optional<uint16_t> maxFFCInterval;

        /// Auto FFC trigger threshold.
        /// The condition for triggering the auto FFC is that the change of Vtemp value exceeds a certain threshold, which is called the Auto FFC trigger
        /// threshold.
        std::optional<uint16_t> autoFFCTempThreshold;

        /// The shutter blade may open/close abnormally during strong mechanical shock (such as fall), and a monitoring process is designed in the firmware to
        /// correct the abnormal shutter switch in time. Turn on or off the fall protect mechanism.
        std::optional<bool> fallProtection;

        /// Frequent FFC will cause shutter heating, resulting in abnormal FFC effect and abnormal temperature measurement.
        /// Regardless of which mechanism triggers FFC, the minimum trigger interval must be limited.
        std::optional<uint16_t> minShutterInterval;

        /// Set this to True/False to close/open the shutter when autoFFC is disabled.
        std::optional<bool> closeManualShutter;

        std::optional<uint16_t> antiFallProtectionThresholdHighGainMode;
        std::optional<uint16_t> antiFallProtectionThresholdLowGainMode;

        DEPTHAI_SERIALIZE(ThermalFFCParams,
                          autoFFC,
                          minFFCInterval,
                          maxFFCInterval,
                          autoFFCTempThreshold,
                          fallProtection,
                          minShutterInterval,
                          closeManualShutter,
                          antiFallProtectionThresholdHighGainMode,
                          antiFallProtectionThresholdLowGainMode);
    };

    struct ThermalImageParams {
        /// 0-3. Time noise filter level. Filters out the noise that appears over time.
        std::optional<uint8_t> timeNoiseFilterLevel;

        /// 0-3. Spatial noise filter level.
        std::optional<uint8_t> spatialNoiseFilterLevel;

        /// 0-4 Digital etail enhance level.
        std::optional<uint8_t> digitalDetailEnhanceLevel;

        /// Image brightness level, 0-255.
        std::optional<uint8_t> brightnessLevel;

        /// Image contrast level, 0-255.
        std::optional<uint8_t> contrastLevel;

        /// Orientation of the image. Computed on the sensor.
        std::optional<ThermalImageOrientation> orientation;
        DEPTHAI_SERIALIZE(
            ThermalImageParams, timeNoiseFilterLevel, spatialNoiseFilterLevel, digitalDetailEnhanceLevel, brightnessLevel, contrastLevel, orientation);
    };

    /// Ambient factors that affect the temperature measurement of a Thermal sensor.
    ThermalAmbientParams ambientParams;

    /// Parameters for Flat-Field-Correction.
    ThermalFFCParams ffcParams;

    /// Image signal processing parameters on the sensor.
    ThermalImageParams imageParams;

    /**
     * Construct ThermalConfig message.
     */
    ThermalConfig() = default;
    virtual ~ThermalConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ThermalConfig;
    }

    DEPTHAI_SERIALIZE(ThermalConfig, ambientParams, ffcParams, imageParams);
};

}  // namespace dai
