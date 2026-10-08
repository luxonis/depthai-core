#pragma once
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Configuration message for time-of-flight depth processing.
 */
class ToFConfig : public Buffer {
   public:
    enum class Profile : uint32_t {
        LOW_RANGE,
        MID_RANGE,
        HIGH_RANGE,
    };

    /**
     * Runtime controls specific to the VD55H1 IPP used by RVC4.
     *
     * An unset value leaves the corresponding device control unchanged. These controls
     * have no effect on RVC2. ToFConfig construction and setProfilePreset() populate
     * all controls; assign VD55H1{} to vd55h1 before setting individual controls
     * when sending a partial update. In Python, use ToFConfig.VD55H1() and None
     * for unset controls.
     *
     * Only phaseUnwrapErrorThreshold has a range declared by the IPP. The limits
     * below for other numeric controls describe their useful input domain; ST does
     * not publish a supported maximum for them.
     */
    struct VD55H1 {
        /** Phase-unwrapping residual threshold in millimeters, in [0, 10000] (IPP range, step 1). Applied at pipeline startup; runtime updates have no effect.
         */
        std::optional<float> phaseUnwrapErrorThreshold;

        /** Enable the bilateral filter (true), or bypass it (false); values are true and false. */
        std::optional<bool> enableBilateralFilter;
        /** Dimensionless standard-deviation multiplier for the bilateral filter; must be > 0 to avoid division by zero. No published maximum. */
        std::optional<float> bilateralStdFactor;

        /** Enable temporal noise reduction (true), or bypass it (false); values are true and false. */
        std::optional<bool> enableTemporalNoiseReduction;
        /** Maximum temporal noise reduction accumulation length, in frames; use >= 1. No published maximum. */
        std::optional<std::uint32_t> temporalNoiseReductionMaxGain;
        /** Dimensionless standard-deviation multiplier for temporal noise rejection; use >= 0. No published maximum. */
        std::optional<float> temporalNoiseReductionStdFactor;

        /** Enable the flying-pixel filter (true), or bypass it (false); values are true and false. */
        std::optional<bool> enableFlyingPixelFilter;
        /** Maximum depth difference between supporting neighboring pixels, in millimeters; use > 0. No published maximum. */
        std::optional<float> flyingPixelDepthThreshold;
        /** Minimum supporting depth-sample count, passed as a float. With the fixed 5x5 neighborhood, use [0, 25); 25 or more rejects every pixel. */
        std::optional<float> flyingPixelMinDepthOccurrence;

        DEPTHAI_SERIALIZE(VD55H1,
                          phaseUnwrapErrorThreshold,
                          enableBilateralFilter,
                          bilateralStdFactor,
                          enableTemporalNoiseReduction,
                          temporalNoiseReductionMaxGain,
                          temporalNoiseReductionStdFactor,
                          enableFlyingPixelFilter,
                          flyingPixelDepthThreshold,
                          flyingPixelMinDepthOccurrence);
    };

    /**
     * Processing controls for the RVC2 S5K33D and S5K63D sensors.
     * These controls have no effect on VD55H1. Corrections use the device defaults
     * determined by the available calibration.
     */
    struct S5K33D {
        /**
         * Phase unwrapping level.
         */
        int phaseUnwrappingLevel = 4;

        /**
         * Phase unwrapping error threshold.
         */
        uint16_t phaseUnwrapErrorThreshold = 100;

        /**
         * Enable phase shuffle temporal filter.
         * Temporal filter that averages the shuffle and non-shuffle frequencies.
         */
        bool enablePhaseShuffleTemporalFilter = true;

        /**
         * Enable burst mode.
         * Decoding is performed on a series of 4 frames.
         * Output fps will be 4 times lower, but reduces motion blur artifacts.
         */
        bool enableBurstMode = false;

        DEPTHAI_SERIALIZE(S5K33D,
                          enablePhaseShuffleTemporalFilter,
                          enableBurstMode,
                          phaseUnwrappingLevel,
                          phaseUnwrapErrorThreshold);
    };

    /** S5K63D uses the same processing controls as S5K33D. */
    using S5K63D = S5K33D;

    /** Deprecated: use s5k33d.phaseUnwrappingLevel (or s5k63d). Unset by default; when set, overrides the nested value with a warning. */
    std::optional<int> phaseUnwrappingLevel = std::nullopt;

    /** Deprecated: use s5k33d.phaseUnwrapErrorThreshold (or s5k63d). Unset by default; when set, overrides the nested value with a warning. */
    std::optional<uint16_t> phaseUnwrapErrorThreshold = std::nullopt;

    /** Deprecated: use s5k33d.enablePhaseShuffleTemporalFilter (or s5k63d). Unset by default; when set, overrides the nested value with a warning. */
    std::optional<bool> enablePhaseShuffleTemporalFilter = std::nullopt;

    /** Deprecated: use s5k33d.enableBurstMode (or s5k63d). Unset by default; when set, overrides the nested value with a warning. */
    std::optional<bool> enableBurstMode = std::nullopt;

    /** Deprecated: this debugging control was removed and is ignored. */
    std::optional<bool> enableFPPNCorrection = std::nullopt;

    /** Deprecated: this debugging control was removed and is ignored. */
    std::optional<bool> enableOpticalCorrection = std::nullopt;

    /** Deprecated: this debugging control was removed and is ignored. */
    std::optional<bool> enableTemperatureCorrection = std::nullopt;

    /** Deprecated: this debugging control was removed and is ignored. */
    std::optional<bool> enableWiggleCorrection = std::nullopt;

    /** Deprecated: this debugging control was removed and is ignored. */
    std::optional<bool> enablePhaseUnwrapping = std::nullopt;

    Profile profile = Profile::MID_RANGE;

    /** Controls for the RVC4 VD55H1 IPP. */
    VD55H1 vd55h1;
    /** Controls for the RVC2 S5K33D and S5K63D sensors. */
    S5K33D s5k33d;

    /** Access the shared Samsung controls under the S5K63D sensor name. */
    S5K63D& s5k63d() {
        return s5k33d;
    }
    const S5K63D& s5k63d() const {
        return s5k33d;
    }

    /**
     * Set kernel size for depth median filtering, or disable
     */
    filters::params::MedianFilter median = filters::params::MedianFilter::MEDIAN_OFF;

    /*
     * Enable distortion correction for intensity, amplitude and depth output, if calibration is present.
     */
    bool enableDistortionCorrection = true;

    /**
     * Construct ToFConfig message.
     */
    ToFConfig() {
        // Preserve the legacy RVC2 default while initializing the RVC4 preset.
        const auto rvc2Threshold = s5k33d.phaseUnwrapErrorThreshold;
        setProfilePreset(profile);
        s5k33d.phaseUnwrapErrorThreshold = rvc2Threshold;
    }
    virtual ~ToFConfig();

    /**
     * @param median Set kernel size for median filtering, or disable
     */
    ToFConfig& setMedianFilter(filters::params::MedianFilter median);

    /** @internal Apply legacy RVC2 fields and report deprecated or removed controls. */
    void applyLegacyConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ToFConfig;
    }

    /**
     * Set preset mode, including the RVC2 phase-unwrapping threshold and VD55H1 processing parameters.
     * @param profile Preset mode for ToFConfig.
     */
    void setProfilePreset(Profile profile);

    DEPTHAI_SERIALIZE(ToFConfig,
                      profile,
                      vd55h1,
                      s5k33d,
                      median,
                      enableDistortionCorrection,
                      phaseUnwrappingLevel,
                      phaseUnwrapErrorThreshold,
                      enablePhaseShuffleTemporalFilter,
                      enableBurstMode,
                      enableFPPNCorrection,
                      enableOpticalCorrection,
                      enableTemperatureCorrection,
                      enableWiggleCorrection,
                      enablePhaseUnwrapping);
};

}  // namespace dai
