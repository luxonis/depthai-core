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
     * Unless stated otherwise, the IPP does not publish a supported numeric range.
     */
    struct VD55H1 {
        /** Phase-unwrapping residual threshold in millimeters, from 0 to 10000. Lower values reject more pixels. */
        std::optional<float> phaseUnwrapErrorThreshold;

        /** Enable the bilateral filter (true), or bypass it (false). */
        std::optional<bool> enableBilateralFilter;
        /** Dimensionless standard-deviation multiplier used by the bilateral filter. */
        std::optional<float> bilateralStdFactor;
        /** Bilateral filter kernel width in pixels. Supported values are odd integers from 3 to 15. */
        std::optional<std::uint32_t> bilateralKernelSize;

        /** Enable temporal noise reduction (true), or bypass it (false). */
        std::optional<bool> enableTemporalNoiseReduction;
        /** Maximum temporal noise reduction accumulation length, in frames. */
        std::optional<std::uint32_t> temporalNoiseReductionMaxGain;
        /** Dimensionless standard-deviation multiplier for temporal noise rejection. */
        std::optional<float> temporalNoiseReductionStdFactor;

        /** Enable the flying-pixel filter (true), or bypass it (false). */
        std::optional<bool> enableFlyingPixelFilter;
        /** Maximum depth difference between supporting neighboring pixels, in millimeters. */
        std::optional<float> flyingPixelDepthThreshold;
        /** Minimum number of neighboring depth samples supporting a pixel; passed to the IPP as a float. */
        std::optional<float> flyingPixelMinDepthOccurrence;

        DEPTHAI_SERIALIZE(VD55H1,
                          phaseUnwrapErrorThreshold,
                          enableBilateralFilter,
                          bilateralStdFactor,
                          bilateralKernelSize,
                          enableTemporalNoiseReduction,
                          temporalNoiseReductionMaxGain,
                          temporalNoiseReductionStdFactor,
                          enableFlyingPixelFilter,
                          flyingPixelDepthThreshold,
                          flyingPixelMinDepthOccurrence);
    };

    Profile profile = Profile::MID_RANGE;

    /** Controls for the RVC4 VD55H1 IPP. */
    VD55H1 vd55h1;
    /**
     * Set kernel size for depth median filtering, or disable
     */
    filters::params::MedianFilter median = filters::params::MedianFilter::MEDIAN_OFF;

    /*
     * Phase unwrapping level.
     */
    int phaseUnwrappingLevel = 4;

    /*
     * RVC2 phase unwrapping error threshold. For VD55H1, use vd55h1.phaseUnwrapErrorThreshold.
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
     * Construct ToFConfig message.
     */
    ToFConfig() {
        // Preserve the legacy RVC2 default while initializing the RVC4 preset.
        const auto rvc2Threshold = phaseUnwrapErrorThreshold;
        setProfilePreset(profile);
        phaseUnwrapErrorThreshold = rvc2Threshold;
    }
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
     * Set preset mode, including the RVC2 phase-unwrapping threshold and VD55H1 processing parameters.
     * @param profile Preset mode for ToFConfig.
     */
    void setProfilePreset(Profile profile);

    DEPTHAI_SERIALIZE(ToFConfig,
                      profile,
                      vd55h1,
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
                      phaseUnwrapErrorThreshold);
};

}  // namespace dai
