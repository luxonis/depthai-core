#pragma once
#include <depthai/common/optional.hpp>
#include <optional>

#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * ThermalConfig message: settings of a Thermal node's sensor. Unset fields keep their current value.
 * Fields a sensor does not support are ignored with a warning.
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

    /// Color palette of the color output. The temperature output is not affected.
    enum class ThermalColorPalette : int32_t { WHITE_HOT, BLACK_HOT, SPECTRA, PRISM, TYRIAN, IRON, AMBER, HI, GREEN };

    /// Image processing pipeline of the Seek SDK.
    enum class ThermalPipelineMode : int32_t {
        /// Minimal image processing filters
        LITE,
        /// Image processing filters of most Seek cores
        LEGACY,
        /// Newest image processing filters and pixel colorization
        SEEKVISION,
        /// Pipeline for Seek Eagle cores
        EAGLE
    };

    /// Automatic gain control: how temperatures are mapped to the color output.
    enum class ThermalAgcMode : int32_t {
        /// Linear between a minimum and a maximum temperature, see ThermalAgcParams::linearLockMode
        LINEAR,
        /// Histogram equalization
        HISTEQ,
        /// Seek's legacy histogram equalization
        LEGACY_HISTEQ
    };

    /// Where the linear AGC takes its temperature range from.
    enum class ThermalLinearAgcLockMode : int32_t {
        /// Minimum and maximum from the scene
        AUTO,
        /// Minimum and maximum from ThermalAgcParams::linearMin and linearMax
        MANUAL,
        /// Minimum from ThermalAgcParams::linearMin, maximum from the scene
        MANUAL_MIN,
        /// Maximum from ThermalAgcParams::linearMax, minimum from the scene
        MANUAL_MAX
    };

    /// AUTO: only ThermalHistEqParams::gainLimit is set by the user. MANUAL: gainLimitFactorYMin and gainLimitFactorXMax as well.
    enum class ThermalHistEqGainLimitFactorMode : int32_t { AUTO, MANUAL };

    /// Where the pixels of histogram bins above the plateau go.
    enum class ThermalHistEqPlateauRedistributionMode : int32_t {
        /// Not redistributed
        DISABLED,
        /// Evenly among all bins
        ALL_BINS,
        /// Evenly among the bins with at least one pixel
        ACTIVE_BINS_ONLY
    };

    /// What to do with the stored flat scene correction.
    enum class ThermalFSCAction : int32_t {
        /// Record it from the frames that follow
        STORE,
        /// Delete it
        CLEAR
    };

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

        /// Constant added to every pixel of the temperature output, in degrees Celsius.
        std::optional<float> temperatureOffset;
        DEPTHAI_SERIALIZE(ThermalAmbientParams,
                          distance,
                          reflectionTemperature,
                          atmosphericTemperature,
                          targetEmissivity,
                          atmosphericTransmittance,
                          gainMode,
                          temperatureOffset);
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

        /// Set to true to run one Flat-Field-Correction (shutter cycle) as soon as possible, with or without autoFFC.
        std::optional<bool> triggerFFC;

        DEPTHAI_SERIALIZE(ThermalFFCParams,
                          autoFFC,
                          minFFCInterval,
                          maxFFCInterval,
                          autoFFCTempThreshold,
                          fallProtection,
                          minShutterInterval,
                          closeManualShutter,
                          antiFallProtectionThresholdHighGainMode,
                          antiFallProtectionThresholdLowGainMode,
                          triggerFFC);
    };

    struct ThermalImageParams {
        /// 0-3. Time noise filter level. Filters out the noise that appears over time.
        std::optional<uint8_t> timeNoiseFilterLevel;

        /// 0-3. Spatial noise filter level.
        std::optional<uint8_t> spatialNoiseFilterLevel;

        /// 0-4 Digital etail enhance level. The Seek SDK only has its sharpen filter: on for any level above 0.
        std::optional<uint8_t> digitalDetailEnhanceLevel;

        /// Image brightness level, 0-255.
        std::optional<uint8_t> brightnessLevel;

        /// Image contrast level, 0-255.
        std::optional<uint8_t> contrastLevel;

        /// Orientation of the image. Computed on the sensor.
        std::optional<ThermalImageOrientation> orientation;

        /// Color palette of the color output. WHITE_HOT (grayscale) at start.
        std::optional<ThermalColorPalette> colorPalette;

        /// Gradient correction filter. It corrects image gradients on flat scenes.
        std::optional<bool> gradientCorrection;

        /// Image processing pipeline.
        std::optional<ThermalPipelineMode> pipelineMode;
        DEPTHAI_SERIALIZE(ThermalImageParams,
                          timeNoiseFilterLevel,
                          spatialNoiseFilterLevel,
                          digitalDetailEnhanceLevel,
                          brightnessLevel,
                          contrastLevel,
                          orientation,
                          colorPalette,
                          gradientCorrection,
                          pipelineMode);
    };

    /**
     * Histogram equalization parameters, used with ThermalAgcMode HISTEQ and LEGACY_HISTEQ.
     */
    struct ThermalHistEqParams {
        /// 0-1. Largest share of the pixels a single histogram bin can hold.
        std::optional<float> plateau;

        /// Where the pixels above the plateau go.
        std::optional<ThermalHistEqPlateauRedistributionMode> plateauRedistributionMode;

        /// 0-256. Largest number of output levels a single histogram bin can take.
        std::optional<float> gainLimit;

        /// Whether gainLimitFactorYMin and gainLimitFactorXMax are set by the user.
        std::optional<ThermalHistEqGainLimitFactorMode> gainLimitFactorMode;

        /// 0-1. Minimum gain limit factor, with gainLimitFactorMode MANUAL.
        std::optional<float> gainLimitFactorYMin;

        /// 0-65535. Histogram width at which the gain limit factor is applied, with gainLimitFactorMode MANUAL.
        std::optional<uint32_t> gainLimitFactorXMax;

        /// 0-6. Time in seconds over which the histogram of each frame is blended with the previous ones; longer is steadier.
        std::optional<float> alphaTimeSeconds;

        /// 0-0.49. Share of the histogram ignored at the cold end.
        std::optional<float> trimLeft;

        /// 0-0.49. Share of the histogram ignored at the hot end.
        std::optional<float> trimRight;

        /// Build the histogram from roi only.
        std::optional<bool> roiEnable;

        /// Histogram region of interest, in pixels or normalized to the output size.
        std::optional<Rect> roi;
        DEPTHAI_SERIALIZE(ThermalHistEqParams,
                          plateau,
                          plateauRedistributionMode,
                          gainLimit,
                          gainLimitFactorMode,
                          gainLimitFactorYMin,
                          gainLimitFactorXMax,
                          alphaTimeSeconds,
                          trimLeft,
                          trimRight,
                          roiEnable,
                          roi);
    };

    /**
     * Automatic gain control: how temperatures are mapped to the color output.
     */
    struct ThermalAgcParams {
        /// AGC mode.
        std::optional<ThermalAgcMode> mode;

        /// Where the linear AGC takes its temperature range from.
        std::optional<ThermalLinearAgcLockMode> linearLockMode;

        /// Linear AGC: temperature of the coldest output level, in degrees Celsius.
        std::optional<float> linearMin;

        /// Linear AGC: temperature of the hottest output level, in degrees Celsius.
        std::optional<float> linearMax;

        /// Histogram equalization parameters.
        ThermalHistEqParams histEq;
        DEPTHAI_SERIALIZE(ThermalAgcParams, mode, linearLockMode, linearMin, linearMax, histEq);
    };

    /**
     * Flat scene correction: corrects the non-uniformity that a window or a housing in front of the sensor adds. It is
     * recorded once from a uniform scene and stored on the device; it is loaded at every pipeline start from then on.
     */
    struct ThermalFSCParams {
        /// Applies the loaded flat scene correction.
        std::optional<bool> enable;

        /// STORE records the correction from the frames that follow: point the camera at a uniform scene, after a warm-up.
        /// It is loaded from the next pipeline start on. CLEAR deletes it. Both pause the outputs for about 2 seconds.
        std::optional<ThermalFSCAction> action;
        DEPTHAI_SERIALIZE(ThermalFSCParams, enable, action);
    };

    /// Ambient factors that affect the temperature measurement of a Thermal sensor.
    ThermalAmbientParams ambientParams;

    /// Parameters for Flat-Field-Correction.
    ThermalFFCParams ffcParams;

    /// Image signal processing parameters on the sensor.
    ThermalImageParams imageParams;

    /// Automatic gain control of the color output.
    ThermalAgcParams agcParams;

    /// Flat scene correction.
    ThermalFSCParams fscParams;

    /**
     * Construct ThermalConfig message.
     */
    ThermalConfig() = default;
    virtual ~ThermalConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ThermalConfig;
    }

    DEPTHAI_SERIALIZE(ThermalConfig, ambientParams, ffcParams, imageParams, agcParams, fscParams);
};

}  // namespace dai
