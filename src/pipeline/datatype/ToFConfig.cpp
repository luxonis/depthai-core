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

}  // namespace dai
