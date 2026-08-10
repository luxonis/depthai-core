#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/XFeatMonoParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief XFeatMonoParser node. Parses the output of the XFeat model from one source (e.g. one camera) into a dai::TrackedFeatures message with the keypoints
 * of a reference frame matched to the keypoints of the current frame.
 *
 * The parser consumes three output tensors, assigned from an NNArchive head by name substring or configured explicitly: the feature map (layer name containing
 * "feats"), read in NCHW orientation as (1, descriptor size, height, width), the keypoint logits (name containing "keypoints"), read as
 * (1, channels >= 64, height, width), and the reliability heat map (name containing "heatmaps"), read as (1, 1, height, width). Every frame is decoded into
 * keypoints, scores and descriptors; the strongest keypoints (up to the maximum count) with a positive score are kept and their positions are scaled from the
 * model input size to the original image size.
 *
 * The parser keeps a reference frame state: calling setTrigger() stores the next decoded result as the reference after that frame's message is emitted.
 * Frames decoded while no reference is stored produce an empty TrackedFeatures message; afterwards each frame's keypoints are matched to the reference by
 * mutual nearest-neighbor cosine similarity and emitted as feature pairs, where match i produces the reference position with id i and age 0 followed by the
 * matched current position with id i and age 1. A frame whose keypoint heat map has no candidate produces an empty message and leaves the reference and the
 * pending trigger untouched.
 *
 * The original image size must be configured before the pipeline starts, either through head metadata or with setOriginalSize().
 */
class XFeatMonoParser : public DeviceNodeCRTP<BetaNode, XFeatMonoParser, XFeatMonoParserProperties> {
   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "XFeatMonoParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    XFeatMonoParser() = default;
    XFeatMonoParser(std::unique_ptr<Properties> props);

    /** Configuration used until a message is received on inputConfig. */
    std::shared_ptr<XFeatMonoParserConfig> initialConfig = std::make_shared<XFeatMonoParserConfig>();

    /**
     * Runtime parser configuration. When synchronized, one configuration is consumed per frame;
     * otherwise all queued configurations are drained and the newest valid one is used.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::XFeatMonoParserConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input NN results with XFeat data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs TrackedFeatures message with the matched keypoint pairs.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::TrackedFeatures, false}}}}};

    /**
     * @brief Build XFeatMonoParser node. Links the supplied output to this node's input and configures the parser from the model's XFeatMonoParser head; when
     * the head metadata carries no input size, the input size is derived from the model input's declared shape and layout.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<XFeatMonoParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build XFeatMonoParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<XFeatMonoParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one XFeatMonoParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive. When the head metadata carries no input size, the input size is derived from the model input's declared shape and layout (NHWC or
     * NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an XFeatMonoParser head with exactly three output layers; the layer whose name contains
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
     * @note Configures startup behavior. Send XFeatMonoParserConfig to inputConfig after the pipeline starts.
     */
    void setMaxKeypoints(int maxKeypoints);

    /**
     * Returns the maximum number of keypoints to keep per frame.
     */
    int getMaxKeypoints() const;

    /**
     * Requests the reference frame update: after the next decoded frame's message is emitted, that frame's keypoints become the reference the following
     * frames are matched against. May be called at any time while the pipeline runs.
     */
    void setTrigger();

    /**
     * Specify whether to run on host or device.
     * By default, the node runs on the device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host.
     */
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = false;
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    std::atomic<bool> trigger{false};
};

}  // namespace node
}  // namespace beta
}  // namespace dai
