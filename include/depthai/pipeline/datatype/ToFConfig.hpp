#pragma once
#include "depthai/common/optional.hpp"
#include "depthai/common/ToFPreset.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * ToFConfig message. Shared wire format for RVC2 decoder and RVC4 IPP settings.
 */
class ToFConfig : public Buffer {
   public:
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
     * Enable bilateral filter inside IPP pipeline.
     * Default: not set (IPP default used). When false, bilateral filtering is bypassed.
     */
    std::optional<bool> enableBilateralFilter;

    /*
     * Bilateral filter standard deviation factor.
     */
    std::optional<float> bilateralStdFactor;

    /*
     * Bilateral filter kernel size.
     */
    std::optional<uint32_t> bilateralFilterKernelSize;

    /*
     * Enable temporal noise reduction inside IPP pipeline.
     * Default: not set (IPP default used). When false, TNR is bypassed.
     */
    std::optional<bool> enableTemporalNoiseReduction;

    /*
     * Temporal noise reduction max gain.
     */
    std::optional<float> tnrMaxGain;

    /*
     * Temporal noise reduction standard deviation factor.
     */
    std::optional<float> tnrStdFactor;

    /*
     * Enable flying pixel correction inside IPP pipeline.
     * Default: not set (IPP default used). When false, flying pixel correction is bypassed.
     */
    std::optional<bool> enableFlyingPixelCorrection;

    /*
     * Flying pixel correction depth threshold.
     */
    std::optional<float> fpDepthThreshold;

    /*
     * Flying pixel correction minimum depth occurrence.
     */
    std::optional<float> fpMinDepthOccurrence;

    /*
     * Enable radial to perpendicular depth conversion inside IPP pipeline.
     * Default: not set (IPP default used). When false, R2P is bypassed.
     */
    std::optional<bool> enableRadialToPerp;

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
    void setProfilePreset(ImageFiltersPresetMode presetMode);

    /**
     * Set RVC4 IPP preset for ToFConfig.
     * @param preset ToF IPP preset mode.
     */
    void setToFPreset(ToFPreset preset);

    DEPTHAI_SERIALIZE(ToFConfig,
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
                      bilateralFilterKernelSize,
                      enableTemporalNoiseReduction,
                      tnrMaxGain,
                      tnrStdFactor,
                      enableFlyingPixelCorrection,
                      fpDepthThreshold,
                      fpMinDepthOccurrence,
                      enableRadialToPerp);
};

/// RVC2 Myriad decoder fields view over the shared ToFConfig wire blob.
struct ToFDecoderConfig {
    filters::params::MedianFilter median = filters::params::MedianFilter::MEDIAN_OFF;
    int phaseUnwrappingLevel = 4;
    uint16_t phaseUnwrapErrorThreshold = 100;
    bool enablePhaseShuffleTemporalFilter = true;
    bool enableBurstMode = false;
    bool enableDistortionCorrection = true;

    std::optional<bool> enableFPPNCorrection;
    std::optional<bool> enableOpticalCorrection;
    std::optional<bool> enableTemperatureCorrection;
    std::optional<bool> enableWiggleCorrection;
    std::optional<bool> enablePhaseUnwrapping;

    static ToFDecoderConfig fromToFConfig(const ToFConfig& config);
    void applyTo(ToFConfig& config) const;
    void applyPreset(ImageFiltersPresetMode presetMode);
};

/// RVC4 IPP fields view over the shared ToFConfig wire blob.
struct ToFIppConfig {
    uint16_t phaseUnwrapErrorThreshold = 100;

    std::optional<bool> enableBilateralFilter;
    std::optional<float> bilateralStdFactor;
    std::optional<uint32_t> bilateralFilterKernelSize;

    std::optional<bool> enableTemporalNoiseReduction;
    std::optional<float> tnrMaxGain;
    std::optional<float> tnrStdFactor;

    std::optional<bool> enableFlyingPixelCorrection;
    std::optional<float> fpDepthThreshold;
    std::optional<float> fpMinDepthOccurrence;

    std::optional<bool> enableRadialToPerp;

    static ToFIppConfig fromToFConfig(const ToFConfig& config);
    void applyTo(ToFConfig& config) const;
    void applyPreset(ToFPreset preset);
};

}  // namespace dai
