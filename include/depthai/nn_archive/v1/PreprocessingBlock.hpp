#pragma once

#include <optional>
#include <string>
#include <vector>

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Preprocessing steps applied to the input data.
 *
 * Represents preprocessing operations applied to the input data.
 *
 * @type mean: list | None
 * @ivar mean: Mean values in channel order. Order depends on the order
 * in which the model was trained on.
 * @type scale: list | None
 * @ivar scale: Standardization values in channel order. Order depends
 * on the order in which the model was trained on.
 * @type reverse_channels: bool | None
 * @ivar reverse_channels: If True input to the model is RGB else BGR.
 * @type interleaved_to_planar: bool | None
 * @ivar interleaved_to_planar: If True input to the model is
 * interleaved (NHWC) else planar (NCHW).
 * @type dai_type: str | None
 * @ivar dai_type: DepthAI input type which is read by DepthAI to
 * automatically setup the pipeline.
 */

/**
 * Preprocessing steps applied to the input data.
 *
 * Represents preprocessing operations applied to the input data.
 *
 * @type mean: list | None
 * @ivar mean: Mean values in channel order. Order depends on the order
 * in which the model was trained on.
 * @type scale: list | None
 * @ivar scale: Standardization values in channel order. Order depends
 * on the order in which the model was trained on.
 * @type reverse_channels: bool | None
 * @ivar reverse_channels: If True input to the model is RGB else BGR.
 * @type interleaved_to_planar: bool | None
 * @ivar interleaved_to_planar: If True input to the model is
 * interleaved (NHWC) else planar (NCHW).
 * @type dai_type: str | None
 * @ivar dai_type: DepthAI input type which is read by DepthAI to
 * automatically setup the pipeline.
 */
struct PreprocessingBlock {
    /**
     * DepthAI input type which is read by DepthAI to automatically setup the pipeline.
     */
    std::optional<std::string> daiType;
    /**
     * If True input to the model is interleaved (NHWC) else planar (NCHW).
     */
    std::optional<bool> interleavedToPlanar;
    /**
     * Mean values in channel order. Order depends on the order in which the model was trained
     * on.
     */
    std::optional<std::vector<double>> mean;
    /**
     * If True input to the model is RGB else BGR.
     */
    std::optional<bool> reverseChannels;
    /**
     * Standardization values in channel order. Order depends on the order in which the model
     * was trained on.
     */
    std::optional<std::vector<double>> scale;
};
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
