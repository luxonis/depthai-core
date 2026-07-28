#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief XFeatStereoParser node. Parses the output of the XFeat model from two sources (e.g. two cameras - left and right) into a dai::TrackedFeatures message
 * with the keypoints of the reference frame matched to the keypoints of the target frame.
 *
 * The parser consumes three output tensors per source, assigned from an NNArchive head by name substring or configured explicitly: the feature map (layer name
 * containing "feats"), read in NCHW orientation as (1, descriptor size, height, width), the keypoint logits (name containing "keypoints"), read as
 * (1, channels >= 64, height, width), and the reliability heat map (name containing "heatmaps"), read as (1, 1, height, width). Both sources share one layer,
 * size and keypoint-count configuration.
 *
 * Each iteration consumes one reference message followed by one target message; the node performs no synchronization beyond these two sequential blocking
 * reads. Both messages are decoded into keypoints, scores and descriptors; the strongest keypoints (up to the maximum count) with a positive score are kept
 * and their positions are scaled from the model input size to the original image size. When the reference (checked first) or the target frame's keypoint heat
 * map has no candidate, an empty TrackedFeatures message is emitted carrying that frame's timestamps and the reference frame's sequence number. Otherwise the
 * reference keypoints are matched to the target keypoints by mutual nearest-neighbor cosine similarity and emitted as feature pairs, where match i produces
 * the reference position with id i and age 0 followed by the matched target position with id i and age 1; the message carries the target frame's timestamps
 * and the reference frame's sequence number, like the source parser.
 *
 * The original image size must be configured before the pipeline starts, either through head metadata or with setOriginalSize().
 *
 * @note This node runs on the host only.
 */
class XFeatStereoParser : public NodeCRTP<dai::node::ThreadedHostNode, XFeatStereoParser> {
   public:
    constexpr static const char* NAME = "XFeatStereoParser";
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results of the reference source (e.g. the left camera) with XFeat data to parse.
     */
    Input referenceInput{*this,
                         {"referenceInput", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input NN results of the target source (e.g. the right camera) with XFeat data to parse.
     */
    Input targetInput{*this, {"targetInput", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs TrackedFeatures message with the matched keypoint pairs.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::TrackedFeatures, false}}}}};

    /**
     * @brief Build XFeatStereoParser node. Links the supplied reference output to this node's referenceInput and the supplied target output to this node's
     * targetInput, and configures the parser from the model's XFeatStereoParser head; when the head metadata carries no input size, the input size is derived
     * from the model input's declared shape and layout.
     * @param reference: Output with the reference source's NN results to link to referenceInput
     * @param target: Output with the target source's NN results to link to targetInput
     * @param model: Neural network model
     */
    std::shared_ptr<XFeatStereoParser> build(Node::Output& reference, Node::Output& target, const Model& model);

    /**
     * @brief Build XFeatStereoParser node with the specific head from an NNArchive. Useful when the model has multiple heads. Links the supplied reference
     * output to this node's referenceInput and the supplied target output to this node's targetInput.
     * @param reference: Output with the reference source's NN results to link to referenceInput
     * @param target: Output with the target source's NN results to link to targetInput
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<XFeatStereoParser> build(Node::Output& reference, Node::Output& target, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one XFeatStereoParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive. When the head metadata carries no input size, the input size is derived from the model input's declared shape and layout (NHWC or
     * NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an XFeatStereoParser head with exactly three output layers; the layer whose name contains
     * "feats" is used as the feature map layer, the layer whose name contains "keypoints" as the keypoint logit layer and the layer whose name contains
     * "heatmaps" as the reliability heat map layer. The original size, input size and maximum keypoint count are read from the head metadata keys
     * original_size, input_size and max_keypoints; missing keys keep the current values.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the output layer containing the feature map.
     *
     * @param outputLayerFeats Name of the output layer containing the feature map
     */
    void setOutputLayerFeats(const std::string& outputLayerFeats);

    /**
     * Returns the name of the output layer containing the feature map.
     */
    std::string getOutputLayerFeats() const;

    /**
     * Sets the name of the output layer containing the keypoint logits.
     *
     * @param outputLayerKeypoints Name of the output layer containing the keypoint logits
     */
    void setOutputLayerKeypoints(const std::string& outputLayerKeypoints);

    /**
     * Returns the name of the output layer containing the keypoint logits.
     */
    std::string getOutputLayerKeypoints() const;

    /**
     * Sets the name of the output layer containing the reliability heat map.
     *
     * @param outputLayerHeatmaps Name of the output layer containing the reliability heat map
     */
    void setOutputLayerHeatmaps(const std::string& outputLayerHeatmaps);

    /**
     * Returns the name of the output layer containing the reliability heat map.
     */
    std::string getOutputLayerHeatmaps() const;

    /**
     * Sets the original image size the emitted keypoint positions are scaled to.
     *
     * Must be configured before the pipeline starts, either explicitly or through head metadata.
     *
     * @param width Original image width, must be greater than 0
     * @param height Original image height, must be greater than 0
     */
    void setOriginalSize(std::uint32_t width, std::uint32_t height);

    /**
     * Returns the original image size as (width, height), or std::nullopt when not configured.
     */
    std::optional<std::pair<std::uint32_t, std::uint32_t>> getOriginalSize() const;

    /**
     * Sets the model input image size the keypoint positions are decoded in.
     *
     * Defaults to 640x352 like the source parser. Configuring from a full NNArchive takes it from the head metadata or derives it from the model input's
     * declared shape and layout; the most recent configuration wins.
     *
     * @param width Input image width, must be greater than 0
     * @param height Input image height, must be greater than 0
     */
    void setInputSize(std::uint32_t width, std::uint32_t height);

    /**
     * Returns the model input image size as (width, height).
     */
    std::pair<std::uint32_t, std::uint32_t> getInputSize() const;

    /**
     * Sets the maximum number of keypoints to keep per frame.
     *
     * @param maxKeypoints Maximum number of keypoints
     */
    void setMaxKeypoints(int maxKeypoints);

    /**
     * Returns the maximum number of keypoints to keep per frame.
     */
    int getMaxKeypoints() const;

    void run() override;

   private:
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    std::string outputLayerFeats = "feats";
    std::string outputLayerKeypoints = "keypoints";
    std::string outputLayerHeatmaps = "heatmaps";
    std::optional<std::pair<std::uint32_t, std::uint32_t>> originalSize;
    std::pair<std::uint32_t, std::uint32_t> inputSize{640, 352};
    int maxKeypoints = 4096;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
