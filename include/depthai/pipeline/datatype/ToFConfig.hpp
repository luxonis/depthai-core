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

    /**
     * ToF illumination pipe type (RVC4 / VD55H1 only).
     */
    enum class PipeType : uint32_t {
        AUTO = 0,   ///< Pick flood/dot from the sensor's detected illumination type.
        FLOOD = 1,  ///< Force flood illumination pipe.
        DOT = 2,    ///< Force dot illumination pipe.
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

    /*
     * RVC4 / VD55H1 depth post-processing tuning.
     *
     * The fields below map directly onto STMicroelectronics VD55H1 IPP controls
     * and are only honored on RVC4 devices. Each is optional: leaving a field
     * unset (std::nullopt) keeps the IPP/wrapper default for that control, so an
     * unmodified ToFConfig reproduces the previous hardcoded behavior.
     */

    /*
     * Bilateral spatial filter. false => filter bypassed.
     */
    std::optional<bool> enableBilateralFilter;
    /*
     * Bilateral filter standard-deviation factor (strength).
     */
    std::optional<float> bilateralStdFactor;
    /*
     * Bilateral filter kernel size.
     */
    std::optional<std::uint32_t> bilateralKernelSize;

    /*
     * Temporal noise reduction (TNR). false => filter bypassed.
     */
    std::optional<bool> enableTemporalNoiseReduction;
    /*
     * TNR maximum gain.
     */
    std::optional<std::uint32_t> tnrMaxGain;
    /*
     * TNR standard-deviation factor (strength).
     */
    std::optional<float> tnrStdFactor;

    /*
     * Flying-pixel filter. false => filter bypassed.
     */
    std::optional<bool> enableFlyingPixelFilter;
    /*
     * Flying-pixel filter depth threshold.
     */
    std::optional<float> flyingPixelDepthThreshold;
    /*
     * Flying-pixel filter minimum depth occurrence: minimum number of neighbouring
     * pixels at a similar depth required for a pixel to be kept (lower => more
     * aggressive removal). Maps to the IPP "min depth occurence" parameter.
     */
    std::optional<float> flyingPixelMinDepthOccurrence;

    /*
     * ToF illumination pipe type. AUTO (default) selects flood/dot from the sensor's
     * detected illumination; FLOOD/DOT force it. Unset keeps the IPP default (AUTO).
     */
    std::optional<PipeType> pipeType;

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
                      enableBilateralFilter,
                      bilateralStdFactor,
                      bilateralKernelSize,
                      enableTemporalNoiseReduction,
                      tnrMaxGain,
                      tnrStdFactor,
                      enableFlyingPixelFilter,
                      flyingPixelDepthThreshold,
                      flyingPixelMinDepthOccurrence,
                      pipeType);
};

}  // namespace dai
