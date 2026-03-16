#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"

namespace dai {

GPUStereoConfig::~GPUStereoConfig() = default;

GPUStereoConfig& GPUStereoConfig::setDepthUnit(AlgorithmControl::DepthUnit depthUnit) {
    algorithmControl.depthUnit = depthUnit;
    return *this;
}

GPUStereoConfig::AlgorithmControl::DepthUnit GPUStereoConfig::getDepthUnit() const {
    return algorithmControl.depthUnit;
}

GPUStereoConfig& GPUStereoConfig::setCustomDepthUnitMultiplier(float multiplier) {
    algorithmControl.customDepthUnitMultiplier = multiplier;
    return *this;
}

float GPUStereoConfig::getCustomDepthUnitMultiplier() const {
    return algorithmControl.customDepthUnitMultiplier;
}

void GPUStereoConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::GPUStereoConfig;
}

}  // namespace dai
