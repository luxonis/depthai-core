#pragma once

#include <stdexcept>

#include "depthai/utility/Serialization.hpp"

namespace dai::beta {

struct ToFStereoFusionConfig {
    float confidenceThreshold = 0.5f;

    ToFStereoFusionConfig& setConfidenceThreshold(float threshold) {
        if(threshold < 0.0f || threshold > 1.0f) throw std::invalid_argument("confidence threshold must be between 0 and 1");
        confidenceThreshold = threshold;
        return *this;
    }
};

DEPTHAI_SERIALIZE_EXT(ToFStereoFusionConfig, confidenceThreshold);

}  // namespace dai::beta
