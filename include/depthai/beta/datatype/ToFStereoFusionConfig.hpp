#pragma once

#include <stdexcept>

#include "depthai/utility/Serialization.hpp"

namespace dai::beta {

struct ToFStereoFusionConfig {
    float confidenceThreshold = 0.5f;

    /** Crop depth and confidence to the largest rectangle in the geometric overlap.
     * Uses viewing directions (projection at infinity), independent of measured depth.
     * Recomputed when input transformations or dimensions change; empty overlap throws.
     */
    bool cropToOverlap = false;

    /**
     * Enable cropping depth and confidence to the largest rectangle inside their geometric overlap.
     * The overlap uses projection at infinity, independently of measured depth. Outputs retain
     * native pixel resolution, with updated intrinsics and unchanged camera extrinsics.
     *
     * @param enabled Whether to crop the outputs. Disabled by default.
     * @return This configuration for chaining.
     * @note Set on initialConfig before starting the pipeline. The crop is computed from the first
     * synchronized input pair and reused until either input transformation or dimensions change.
     * If the geometric overlap is empty, the node throws an error while processing that pair.
     */
    ToFStereoFusionConfig& setCropToOverlap(bool enabled) {
        cropToOverlap = enabled;
        return *this;
    }

    ToFStereoFusionConfig& setConfidenceThreshold(float threshold) {
        if(threshold < 0.0f || threshold > 1.0f) throw std::invalid_argument("confidence threshold must be between 0 and 1");
        confidenceThreshold = threshold;
        return *this;
    }
};

DEPTHAI_SERIALIZE_EXT(ToFStereoFusionConfig, confidenceThreshold, cropToOverlap);

}  // namespace dai::beta
