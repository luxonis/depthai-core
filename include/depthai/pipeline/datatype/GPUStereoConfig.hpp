#pragma once

#include <cstdint>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * @brief Configuration for the GPUStereo node.
 */
class GPUStereoConfig : public Buffer {
   public:
    /**
     * Where the stereo matching runs.
     *  - GPU_CPU (default): vertical SGM paths on the GPU, horizontal paths on one CPU core (lowest GPU load).
     *  - GPU_ONLY: everything on the GPU (lowest CPU load).
     * Both produce the same output.
     */
    enum class ComputeMode : std::int32_t { GPU_CPU = 0, GPU_ONLY = 1 };

    /**
     * @brief Confidence threshold for disparity filtering.
     *
     * Value in range [0, 255]. 0 disables the filter. Pixels whose confidence (as in the confidenceMap
     * output, higher = more confident) is below the threshold are invalidated in disparity and depth.
     * Raising the threshold lowers the fill rate. The scale is specific to this node (not comparable value
     * for value with StereoDepth / NeuralDepth); typical fill on indoor scenes: 0 -> ~98 %, 10 (default) -> ~79 %,
     * 13 -> ~74 %, 25 -> ~55 %, 55 -> ~29 %.
     */
    std::uint8_t confidenceThreshold = 10;

    /**
     * Compute placement, see ComputeMode. Applied at startup (initialConfig).
     */
    ComputeMode computeMode = ComputeMode::GPU_CPU;

    /**
     * Temporal filter (default on): each frame is blended with the node's previous output. Pixels that agree
     * with it are smoothed and gain confidence (steadier, fuller depth on static scenes); pixels that disagree
     * (motion) keep their single-frame confidence. Off: every frame is independent. Applied at startup.
     */
    bool temporalFilter = true;

    GPUStereoConfig() = default;
    virtual ~GPUStereoConfig();

    /**
     * @brief Set the confidence threshold for disparity filtering.
     *
     * Pixels with a confidence below this threshold are invalidated.
     * @param threshold Value in range [0, 255]. 0 disables the filter. Values outside the range are clamped.
     */
    GPUStereoConfig& setConfidenceThreshold(int threshold);

    /**
     * @brief Get the confidence threshold for disparity filtering.
     */
    int getConfidenceThreshold() const;

    /**
     * @brief Set where the stereo matching runs (GPU_CPU default, or GPU_ONLY).
     */
    GPUStereoConfig& setComputeMode(ComputeMode mode);

    /**
     * @brief Get the compute placement.
     */
    ComputeMode getComputeMode() const;

    /**
     * @brief Enable or disable the temporal filter (default enabled).
     */
    GPUStereoConfig& setTemporalFilter(bool enable);

    /**
     * @brief Whether the temporal filter is enabled.
     */
    bool getTemporalFilter() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(GPUStereoConfig, confidenceThreshold, computeMode, temporalFilter);
};

}  // namespace dai
