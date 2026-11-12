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

void ToFConfig::setProfilePreset(Profile prof) {
    profile = prof;
    switch(prof) {
        case Profile::LOW_RANGE: {
            this->phaseUnwrapErrorThreshold = 50;
        } break;
        case Profile::MID_RANGE: {
            this->phaseUnwrapErrorThreshold = 75;
        } break;
        case Profile::HIGH_RANGE: {
            this->phaseUnwrapErrorThreshold = 130;
        } break;
    }
}

}  // namespace dai
