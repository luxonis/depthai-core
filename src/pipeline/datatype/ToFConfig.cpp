#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

ToFConfig::~ToFConfig() = default;

void ToFConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ToFConfig;
}

ToFConfig& ToFConfig::setMedianFilter(filters::params::MedianFilter median) {
    this->median = median;
    return *this;
}

void ToFConfig::setProfilePreset(ImageFiltersPresetMode presetMode) {
    switch(presetMode) {
        case ImageFiltersPresetMode::TOF_LOW_RANGE: {
            this->phaseUnwrapErrorThreshold = 50;
        } break;
        case ImageFiltersPresetMode::TOF_MID_RANGE: {
            this->phaseUnwrapErrorThreshold = 75;
        } break;
        case ImageFiltersPresetMode::TOF_HIGH_RANGE: {
            this->phaseUnwrapErrorThreshold = 130;
        } break;
        case ImageFiltersPresetMode::TOF_OFF: {
            this->phaseUnwrapErrorThreshold = 10000;
        } break;
    }
    // IPP filter fields (std::optional): when unset, IPP keeps its own built-in
    // defaults — which have bilateral, TNR, flying-pixel and R2P all ENABLED.
    // Only the host-side ImageFilters node is controlled separately via TOF_OFF preset.
}

void ToFConfig::setToFPreset(ToFPreset preset) {
    ToFIppConfig ipp;
    ipp.applyPreset(preset);
    ipp.applyTo(*this);
}

ToFDecoderConfig ToFDecoderConfig::fromToFConfig(const ToFConfig& config) {
    ToFDecoderConfig decoder;
    decoder.median = config.median;
    decoder.phaseUnwrappingLevel = config.phaseUnwrappingLevel;
    decoder.phaseUnwrapErrorThreshold = config.phaseUnwrapErrorThreshold;
    decoder.enablePhaseShuffleTemporalFilter = config.enablePhaseShuffleTemporalFilter;
    decoder.enableBurstMode = config.enableBurstMode;
    decoder.enableDistortionCorrection = config.enableDistortionCorrection;
    decoder.enableFPPNCorrection = config.enableFPPNCorrection;
    decoder.enableOpticalCorrection = config.enableOpticalCorrection;
    decoder.enableTemperatureCorrection = config.enableTemperatureCorrection;
    decoder.enableWiggleCorrection = config.enableWiggleCorrection;
    decoder.enablePhaseUnwrapping = config.enablePhaseUnwrapping;
    return decoder;
}

void ToFDecoderConfig::applyTo(ToFConfig& config) const {
    config.median = median;
    config.phaseUnwrappingLevel = phaseUnwrappingLevel;
    config.phaseUnwrapErrorThreshold = phaseUnwrapErrorThreshold;
    config.enablePhaseShuffleTemporalFilter = enablePhaseShuffleTemporalFilter;
    config.enableBurstMode = enableBurstMode;
    config.enableDistortionCorrection = enableDistortionCorrection;
    config.enableFPPNCorrection = enableFPPNCorrection;
    config.enableOpticalCorrection = enableOpticalCorrection;
    config.enableTemperatureCorrection = enableTemperatureCorrection;
    config.enableWiggleCorrection = enableWiggleCorrection;
    config.enablePhaseUnwrapping = enablePhaseUnwrapping;
}

void ToFDecoderConfig::applyPreset(ImageFiltersPresetMode presetMode) {
    ToFConfig config;
    config.setProfilePreset(presetMode);
    phaseUnwrapErrorThreshold = config.phaseUnwrapErrorThreshold;
}

ToFIppConfig ToFIppConfig::fromToFConfig(const ToFConfig& config) {
    ToFIppConfig ipp;
    ipp.phaseUnwrapErrorThreshold = config.phaseUnwrapErrorThreshold;
    ipp.enableBilateralFilter = config.enableBilateralFilter;
    ipp.bilateralStdFactor = config.bilateralStdFactor;
    ipp.bilateralFilterKernelSize = config.bilateralFilterKernelSize;
    ipp.enableTemporalNoiseReduction = config.enableTemporalNoiseReduction;
    ipp.tnrMaxGain = config.tnrMaxGain;
    ipp.tnrStdFactor = config.tnrStdFactor;
    ipp.enableFlyingPixelCorrection = config.enableFlyingPixelCorrection;
    ipp.fpDepthThreshold = config.fpDepthThreshold;
    ipp.fpMinDepthOccurrence = config.fpMinDepthOccurrence;
    ipp.enableRadialToPerp = config.enableRadialToPerp;
    return ipp;
}

void ToFIppConfig::applyTo(ToFConfig& config) const {
    config.phaseUnwrapErrorThreshold = phaseUnwrapErrorThreshold;
    config.enableBilateralFilter = enableBilateralFilter;
    config.bilateralStdFactor = bilateralStdFactor;
    config.bilateralFilterKernelSize = bilateralFilterKernelSize;
    config.enableTemporalNoiseReduction = enableTemporalNoiseReduction;
    config.tnrMaxGain = tnrMaxGain;
    config.tnrStdFactor = tnrStdFactor;
    config.enableFlyingPixelCorrection = enableFlyingPixelCorrection;
    config.fpDepthThreshold = fpDepthThreshold;
    config.fpMinDepthOccurrence = fpMinDepthOccurrence;
    config.enableRadialToPerp = enableRadialToPerp;
}

void ToFIppConfig::applyPreset(ToFPreset preset) {
    // Reset every tuning field so applyPreset() is deterministic regardless of the
    // object's prior state (e.g. when called on a ToFIppConfig::fromToFConfig(...) result).
    // Without this, stale bilateral/TNR/flying-pixel tuning values would leak back into ToFConfig.
    bilateralStdFactor = std::nullopt;
    bilateralFilterKernelSize = std::nullopt;
    tnrMaxGain = std::nullopt;
    tnrStdFactor = std::nullopt;
    fpDepthThreshold = std::nullopt;
    fpMinDepthOccurrence = std::nullopt;

    switch(preset) {
        case ToFPreset::LOW_RANGE:
            phaseUnwrapErrorThreshold = 50;
            enableBilateralFilter = std::nullopt;
            enableTemporalNoiseReduction = std::nullopt;
            enableFlyingPixelCorrection = std::nullopt;
            enableRadialToPerp = std::nullopt;
            break;
        case ToFPreset::MID_RANGE:
            phaseUnwrapErrorThreshold = 75;
            enableBilateralFilter = std::nullopt;
            enableTemporalNoiseReduction = std::nullopt;
            enableFlyingPixelCorrection = std::nullopt;
            enableRadialToPerp = std::nullopt;
            break;
        case ToFPreset::HIGH_RANGE:
            phaseUnwrapErrorThreshold = 130;
            enableBilateralFilter = std::nullopt;
            enableTemporalNoiseReduction = std::nullopt;
            enableFlyingPixelCorrection = std::nullopt;
            enableRadialToPerp = std::nullopt;
            break;
        case ToFPreset::FAST_OBJECTS:
            // Tuned for fast-moving scenes: disable temporal noise reduction to cut motion
            // blur/ghosting; keep other IPP filters at their IPP defaults. Exact tuning TBD.
            phaseUnwrapErrorThreshold = 75;
            enableBilateralFilter = std::nullopt;
            enableTemporalNoiseReduction = false;
            enableFlyingPixelCorrection = std::nullopt;
            enableRadialToPerp = std::nullopt;
            break;
        case ToFPreset::OFF:
            phaseUnwrapErrorThreshold = 10000;
            enableBilateralFilter = false;
            enableTemporalNoiseReduction = false;
            enableFlyingPixelCorrection = false;
            enableRadialToPerp = false;
            break;
    }
}

}  // namespace dai
