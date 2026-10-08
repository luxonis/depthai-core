#include "depthai/pipeline/datatype/ToFConfig.hpp"

#include <spdlog/spdlog.h>

namespace dai {

ToFConfig::~ToFConfig() = default;

void ToFConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    auto config = *this;
    config.applyLegacyConfig();
    metadata = utility::serialize(config);
    datatype = DatatypeEnum::ToFConfig;
}

void ToFConfig::applyLegacyConfig() {
    if(phaseUnwrappingLevel.has_value()) {
        spdlog::warn("ToFConfig.phaseUnwrappingLevel is deprecated; use ToFConfig.s5k33d.phaseUnwrappingLevel (or s5k63d) instead.");
        s5k33d.phaseUnwrappingLevel = *phaseUnwrappingLevel;
    }
    if(phaseUnwrapErrorThreshold.has_value()) {
        spdlog::warn("ToFConfig.phaseUnwrapErrorThreshold is deprecated; use ToFConfig.s5k33d.phaseUnwrapErrorThreshold (or s5k63d) instead.");
        s5k33d.phaseUnwrapErrorThreshold = *phaseUnwrapErrorThreshold;
    }
    if(enablePhaseShuffleTemporalFilter.has_value()) {
        spdlog::warn("ToFConfig.enablePhaseShuffleTemporalFilter is deprecated; use ToFConfig.s5k33d.enablePhaseShuffleTemporalFilter (or s5k63d) instead.");
        s5k33d.enablePhaseShuffleTemporalFilter = *enablePhaseShuffleTemporalFilter;
    }
    if(enableBurstMode.has_value()) {
        spdlog::warn("ToFConfig.enableBurstMode is deprecated; use ToFConfig.s5k33d.enableBurstMode (or s5k63d) instead.");
        s5k33d.enableBurstMode = *enableBurstMode;
    }
    if(enableFPPNCorrection.has_value()) {
        spdlog::warn("ToFConfig.enableFPPNCorrection is deprecated: this control was removed and its value is ignored.");
    }
    if(enableOpticalCorrection.has_value()) {
        spdlog::warn("ToFConfig.enableOpticalCorrection is deprecated: this control was removed and its value is ignored.");
    }
    if(enableTemperatureCorrection.has_value()) {
        spdlog::warn("ToFConfig.enableTemperatureCorrection is deprecated: this control was removed and its value is ignored.");
    }
    if(enableWiggleCorrection.has_value()) {
        spdlog::warn("ToFConfig.enableWiggleCorrection is deprecated: this control was removed and its value is ignored.");
    }
    if(enablePhaseUnwrapping.has_value()) {
        spdlog::warn("ToFConfig.enablePhaseUnwrapping is deprecated: this control was removed and its value is ignored.");
    }
}

ToFConfig& ToFConfig::setMedianFilter(filters::params::MedianFilter median) {
    this->median = median;
    return *this;
}

void ToFConfig::setProfilePreset(Profile prof) {
    profile = prof;
    switch(prof) {
        case Profile::LOW_RANGE: {
            s5k33d.phaseUnwrapErrorThreshold = 50;
            vd55h1 = {82.0f, true, 7.266f, true, 1, 0.9039f, true, 191.3f, 14.95f};
        } break;
        case Profile::MID_RANGE: {
            s5k33d.phaseUnwrapErrorThreshold = 75;
            vd55h1 = {192.0f, true, 2.051f, true, 27, 0.8205f, true, 100.9f, 13.56f};
        } break;
        case Profile::HIGH_RANGE: {
            s5k33d.phaseUnwrapErrorThreshold = 130;
            vd55h1 = {300.0f, true, 2.051f, true, 27, 0.8205f, true, 100.9f, 13.56f};
        } break;
    }
}

}  // namespace dai
