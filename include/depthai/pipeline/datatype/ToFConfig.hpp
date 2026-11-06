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
     * Publish intermediate depth updates on RVC2 S5K63D when phase unwrapping
     * is enabled. These updates combine the new frequency pair with
     * opposite-frequency history from the previous four-frame stack.
     * Disabled by default: only the depth result using both frequency pairs
     * from the current stack is published. Burst mode suppresses intermediate
     * updates regardless of this setting. Other sensors are unaffected.
     */
    bool enableIntermediateDepth = false;

    /**
     * Construct ToFConfig message.
     */
    ToFConfig() = default;
    virtual ~ToFConfig();

    /**
     * @param median Set kernel size for median filtering, or disable
     */
    ToFConfig& setMedianFilter(filters::params::MedianFilter median);

    /**
     * Select whether to publish intermediate S5K63D depth updates.
     * May be used with initialConfig or sent through inputConfig at runtime.
     * This changes output selection, not calibration corrections.
     * @param enable True to publish intermediate updates; false by default.
     * @return This configuration.
     * @see enableIntermediateDepth
     */
    ToFConfig& setEnableIntermediateDepth(bool enable);

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
                      enableIntermediateDepth);
};

}  // namespace dai
