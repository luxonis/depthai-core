#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"

#include <algorithm>

namespace dai {

GPUStereoConfig::~GPUStereoConfig() = default;

GPUStereoConfig& GPUStereoConfig::setConfidenceThreshold(int threshold) {
    confidenceThreshold = static_cast<std::uint8_t>(std::clamp(threshold, 0, 255));
    return *this;
}

int GPUStereoConfig::getConfidenceThreshold() const {
    return confidenceThreshold;
}

GPUStereoConfig& GPUStereoConfig::setComputeMode(ComputeMode mode) {
    computeMode = mode;
    return *this;
}

GPUStereoConfig::ComputeMode GPUStereoConfig::getComputeMode() const {
    return computeMode;
}

GPUStereoConfig& GPUStereoConfig::setTemporalFilter(bool enable) {
    temporalFilter = enable;
    return *this;
}

bool GPUStereoConfig::getTemporalFilter() const {
    return temporalFilter;
}

void GPUStereoConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::GPUStereoConfig;
}

}  // namespace dai
