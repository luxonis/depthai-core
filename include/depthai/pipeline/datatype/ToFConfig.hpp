#pragma once
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * ToFConfig message. Carries config for feature tracking algorithm
 */
class ToFConfig : public Buffer {
   public:
    enum class Profile : uint32_t {
        LOW_RANGE,
        MID_RANGE,
        HIGH_RANGE,
    };

    Profile profile = Profile::MID_RANGE;
    /**
     * Set kernel size for depth median filtering, or disable
     */
    filters::params::MedianFilter median = filters::params::MedianFilter::MEDIAN_OFF;

    /*
     * Phase unwrapping level.
     */
    int phaseUnwrappingLevel = 4;

    /*
     * Phase unwrapping error threshold.
     */
    uint16_t phaseUnwrapErrorThreshold = 100;

    /*
     * Enable phase shuffle temporal filter.
     * Temporal filter that averages the shuffle and non-shuffle frequencies.
     */
    bool enablePhaseShuffleTemporalFilter = true;

    /*
     * Enable burst mode.
     * Decoding is performed on a series of 4 frames.
     * Output fps will be 4 times lower, but reduces motion blur artifacts.
     */
    bool enableBurstMode = false;

    /*
     * Enable distortion correction for intensity, amplitude and depth output, if calibration is present.
     */
    bool enableDistortionCorrection = true;

    /*
     * Enable FPN correction. Used for debugging.
     */
    std::optional<bool> enableFPPNCorrection;
    /*
     * Enable optical correction. Used for debugging.
     */
    std::optional<bool> enableOpticalCorrection;
    /*
     * Enable temperature correction. Used for debugging.
     */
    std::optional<bool> enableTemperatureCorrection;
    /*
     * Enable wiggle correction. Used for debugging.
     */
    std::optional<bool> enableWiggleCorrection;
    /*
     * Enable phase unwrapping. Used for debugging.
     */
    std::optional<bool> enablePhaseUnwrapping;

    /**
     * Depth processing mode. Applies to RVC4; other platforms ignore it.
     */
    enum class DepthMode : uint32_t {
        /**
         * Depth as selected by the profile and phaseUnwrappingLevel (default).
         */
        STANDARD,
        /**
         * Depth for ToF + stereo fusion networks. Uses the spatial phase unwrap
         * (phaseUnwrappingLevel 103) whatever phaseUnwrappingLevel is set to, with
         * dense output: every pixel with a valid depth up to 15 m is kept, there is
         * no confidence threshold, no edge filter, no 6.5 m range cap and no bleeding
         * filter. The confidence output is calibrated as
         * round(255 * (1 - P(|error| > 0.5 m))). The sensor filters use the tuning
         * fusion networks are trained on: no phase-unwrap error rejection, no
         * flying-pixel filter, bilateral and temporal noise reduction on.
         * When the phase output is linked, it carries the wrapped phases of the three
         * modulation frequencies: RAW8, width = 6 * depth width, per depth pixel
         * int8 (127 * cos, 127 * sin) for each frequency.
         * Without a usable GPU the node falls back to the vendor unwrap with the same
         * filter tuning and sends no phase frames.
         */
        FUSION,
    };

    /**
     * Depth processing mode, see DepthMode.
     */
    DepthMode depthMode = DepthMode::STANDARD;

    /**
     * Construct ToFConfig message.
     */
    ToFConfig() = default;
    virtual ~ToFConfig();

    /**
     * @param median Set kernel size for median filtering, or disable
     */
    ToFConfig& setMedianFilter(filters::params::MedianFilter median);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ToFConfig;
    }

    /**
     * Set preset mode for ToFConfig.
     * @param presetMode Preset mode for ToFConfig.
     */
    void setProfilePreset(Profile profile);

    /**
     * Set the depth processing mode.
     * May be used with initialConfig or sent through inputConfig at runtime.
     * @param mode Depth processing mode, see DepthMode.
     * @return This configuration.
     */
    ToFConfig& setDepthMode(DepthMode mode);

    DEPTHAI_SERIALIZE(ToFConfig,
                      profile,
                      median,
                      enablePhaseShuffleTemporalFilter,
                      enableBurstMode,
                      enableDistortionCorrection,
                      enableFPPNCorrection,
                      enableOpticalCorrection,
                      enableTemperatureCorrection,
                      enableWiggleCorrection,
                      enablePhaseUnwrapping,
                      phaseUnwrappingLevel,
                      phaseUnwrapErrorThreshold,
                      depthMode);
};

}  // namespace dai
