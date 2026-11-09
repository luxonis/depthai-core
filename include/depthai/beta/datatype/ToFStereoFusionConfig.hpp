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

    /**
     * Set the confidence threshold.
     *
     * With the ALPHA_BLEND model a pixel is kept when its fused confidence is at least the threshold.
     * With the STEREO_NETWORK model the threshold grows linearly with depth: a pixel at depth Z is kept
     * when its confidence is at least threshold * Z / 10 m, so the value is the threshold at 10 m.
     *
     * @param threshold Threshold in [0, 1].
     * @return This configuration for chaining.
     */
    ToFStereoFusionConfig& setConfidenceThreshold(float threshold) {
        if(threshold < 0.0f || threshold > 1.0f) throw std::invalid_argument("confidence threshold must be between 0 and 1");
        confidenceThreshold = threshold;
        return *this;
    }

    /**
     * STEREO_NETWORK model only: output the ToF depth directly at close range.
     *
     * The network cannot output depth below its largest disparity (about 0.33 m at 1280x800 on OAK4-LITE-TOF).
     * When enabled, a pixel takes the ToF depth and skips the confidence threshold when both the ToF depth is
     * below nearRangeTofMaxDepth and the fused depth is below nearRangeFusedMaxDepth. ToF samples closer than
     * nearRangeMinDepth are ignored. The network inputs do not change.
     */
    bool nearRangePassthrough = false;
    /** Closest ToF sample used by the near-range passthrough [mm]. */
    float nearRangeMinDepth = 150.0f;
    /** The near-range passthrough applies where the ToF depth is below this value [mm]. */
    float nearRangeTofMaxDepth = 343.0f;
    /** The near-range passthrough applies where the fused depth is below this value [mm]. */
    float nearRangeFusedMaxDepth = 418.0f;

    /**
     * Enable or disable the near-range ToF passthrough of the STEREO_NETWORK model.
     *
     * @param enabled Whether to output the ToF depth at close range.
     * @param minDepth Closest ToF sample to use [mm].
     * @param tofMaxDepth The passthrough applies where the ToF depth is below this value [mm].
     * @param fusedMaxDepth The passthrough applies where the fused depth is below this value [mm].
     * @return This configuration for chaining.
     */
    ToFStereoFusionConfig& setNearRangePassthrough(bool enabled, float minDepth = 150.0f, float tofMaxDepth = 343.0f, float fusedMaxDepth = 418.0f) {
        if(!(minDepth > 0.0f) || !(tofMaxDepth > minDepth) || !(fusedMaxDepth > 0.0f)) {
            throw std::invalid_argument("near-range depths must be positive and tofMaxDepth must be greater than minDepth");
        }
        nearRangePassthrough = enabled;
        nearRangeMinDepth = minDepth;
        nearRangeTofMaxDepth = tofMaxDepth;
        nearRangeFusedMaxDepth = fusedMaxDepth;
        return *this;
    }
};

DEPTHAI_SERIALIZE_EXT(
    ToFStereoFusionConfig, confidenceThreshold, cropToOverlap, nearRangePassthrough, nearRangeMinDepth, nearRangeTofMaxDepth, nearRangeFusedMaxDepth);

}  // namespace dai::beta
