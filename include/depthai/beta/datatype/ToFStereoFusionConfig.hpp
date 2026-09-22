#pragma once

#include <stdexcept>

#include "depthai/utility/Serialization.hpp"

namespace dai::beta {

struct ToFStereoFusionConfig {
    /** Crop depth and confidence to the largest rectangle in the geometric overlap.
     * Uses viewing directions (projection at infinity), independent of measured depth.
     * Recomputed when input transformations or dimensions change; empty overlap throws.
     */
    bool cropToOverlap = false;

    ToFStereoFusionConfig& setCropToOverlap(bool enabled) {
        cropToOverlap = enabled;
        return *this;
    }

    float confidenceThreshold = 0.5f;

    ToFStereoFusionConfig& setConfidenceThreshold(float threshold) {
        if(threshold < 0.0f || threshold > 1.0f) throw std::invalid_argument("confidence threshold must be between 0 and 1");
        confidenceThreshold = threshold;
        return *this;
    }
};

DEPTHAI_SERIALIZE_EXT(ToFStereoFusionConfig, confidenceThreshold, cropToOverlap);

}  // namespace dai::beta
