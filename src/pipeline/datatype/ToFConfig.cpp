#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

ToFConfig& ToFConfig::setMedianFilter(MedianFilter median) {
    this->median = median;
    return *this;
}

void ToFConfig::setProfilePreset(ImageFiltersPresetMode presetMode) {
    switch(presetMode) {
        case ImageFiltersPresetMode::LOW_RANGE: {
            this->phaseUnwrapErrorThreshold = 50;
        } break;
        case ImageFiltersPresetMode::MID_RANGE: {
            this->phaseUnwrapErrorThreshold = 75;
        } break;
        case ImageFiltersPresetMode::HIGH_RANGE: {
            this->phaseUnwrapErrorThreshold = 130;
        } break;
    }
}

}  // namespace dai
