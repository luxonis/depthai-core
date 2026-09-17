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
            vd55h1 = {82.0f, true, 7.266f, 5, true, 1, 0.9039f, true, 191.3f, 14.95f, std::nullopt};
        } break;
        case Profile::MID_RANGE: {
            vd55h1 = {192.0f, true, 2.051f, 5, true, 27, 0.8205f, true, 100.9f, 13.56f, std::nullopt};
        } break;
        case Profile::HIGH_RANGE: {
            vd55h1 = {300.0f, true, 2.051f, 5, true, 27, 0.8205f, true, 100.9f, 13.56f, std::nullopt};
        } break;
    }
}

}  // namespace dai
