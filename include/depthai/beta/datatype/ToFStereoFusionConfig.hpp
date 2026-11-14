#pragma once

#include <stdexcept>

#include "depthai/utility/Serialization.hpp"

namespace dai::beta {

struct ToFStereoFusionConfig {
    /**
     * Confidence threshold in [0, 1], mapped to a threshold on the published u8 confidence: the default 0.5 maps to the
     * calibrated default of the network, other values proportionally (0 keeps every pixel). Depth is kept where the
     * confidence is at or above the mapped threshold.
     */
    float confidenceThreshold = 0.5f;

    /** Crop depth and confidence to the largest rectangle of the rectified-left view that the ToF field of view covers.
     * Uses viewing directions (projection at infinity), independent of measured depth: at close range the ToF covers a few
     * columns less on the side away from it (about 40 px at infinity and 86 px at 0.3 m of the 1024 px width on OAK4-LITE-TOF).
     * The crop is added to the output transformation (shifted intrinsics). Recomputed when an input transformation or size
     * changes; without an overlap the outputs stay uncropped (with a warning).
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
     * Without a geometric overlap the outputs stay uncropped and the node logs a warning.
     */
    ToFStereoFusionConfig& setCropToOverlap(bool enabled) {
        cropToOverlap = enabled;
        return *this;
    }

    /**
     * Set the confidence threshold, see confidenceThreshold.
     *
     * @param threshold Threshold in [0, 1]; 0.5 (default) is the calibrated default of the network.
     * @return This configuration for chaining.
     */
    ToFStereoFusionConfig& setConfidenceThreshold(float threshold) {
        if(threshold < 0.0f || threshold > 1.0f) throw std::invalid_argument("confidence threshold must be between 0 and 1");
        confidenceThreshold = threshold;
        return *this;
    }
};

DEPTHAI_SERIALIZE_EXT(ToFStereoFusionConfig, confidenceThreshold, cropToOverlap);

}  // namespace dai::beta
