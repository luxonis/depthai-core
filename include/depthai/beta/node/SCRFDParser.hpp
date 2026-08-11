#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/SCRFDParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief SCRFDParser node. Parses the output of SCRFD detection models (e.g. SCRFD face and person detection) into a dai::ImgDetections message containing the
 * bounding boxes, labels, confidence scores and 5 keypoints per detected object, everything normalized to [0, 1].
 *
 * The parser consumes three output tensors per configured feature stride, named score_{stride}, bbox_{stride} and kps_{stride}. The score tensor is flattened,
 * the bbox tensor is paired 4 values per score (left, top, right, bottom distances from the anchor center) and the kps tensor 10 values per score (5 keypoint
 * coordinate pairs), accepting both batched (1, N, C) and unbatched (N, C) tensors. Scores greater than or equal to the confidence threshold (inclusive) are
 * kept, decoded against the anchor centers derived from the input size, the stride and the number of anchors, sorted by descending score and suppressed with
 * the original SCRFD non-maximum suppression (+1 offset box areas, overlaps at most the IoU threshold survive). The anchor centers are cached across messages
 * and refreshed when the input size, feature strides or number of anchors change.
 *
 */
class SCRFDParser : public DeviceNodeCRTP<BetaNode, SCRFDParser, SCRFDParserProperties> {
   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "SCRFDParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    SCRFDParser() = default;
    SCRFDParser(std::unique_ptr<Properties> props);

    /**
     * Configuration used when the parser starts.
     */
    std::shared_ptr<SCRFDParserConfig> initialConfig = std::make_shared<SCRFDParserConfig>();

    /**
     * Runtime parser configuration. In synchronized mode one configuration is consumed per input frame;
     * otherwise all queued configurations are drained and the newest valid one is retained.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::SCRFDParserConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input NN results with SCRFD detection data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgDetections message with the bounding boxes, labels, confidence scores and keypoints of the detected objects.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * @brief Build SCRFDParser node. Links the supplied output to this node's input and configures the parser from the model's SCRFDParser head and the model
     * input's declared shape and layout.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<SCRFDParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build SCRFDParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     * @note A head carries no model input metadata, so the input size keeps its current value; configure it with setInputSize() when it differs from the
     * default.
     */
    std::shared_ptr<SCRFDParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one SCRFDParser head and exactly one model input; use setNNArchiveHead() to select
     * a specific head from a multi-head archive. The input size is derived from the model input's declared shape and layout (NHWC or NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an SCRFDParser head with an equal number of score, bbox and kps output layers.
     *
     * @param head: NNArchive head to set
     * @note A head carries no model input metadata, so the input size keeps its current value; configure it with setInputSize() when it differs from the
     * default.
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the names of the model output layers relevant to the parser. The parser looks up the per-stride score_{stride}, bbox_{stride} and kps_{stride}
     * layer names in this list.
     *
     * When left empty, the list is resolved from the layer names of the first incoming NNData.
     *
     * @param outputLayerNames Names of the output layers
     */
    void setOutputLayerNames(const std::vector<std::string>& outputLayerNames);

    /**
     * Returns the names of the model output layers relevant to the parser.
     */
    std::vector<std::string> getOutputLayerNames() const;

    /**
     * Sets the confidence score threshold for detected objects. Detections with a score greater than or equal to the threshold (inclusive) are kept.
     *
     * @param threshold Confidence score threshold
     * @note Configures startup behavior. Send SCRFDParserConfig to inputConfig after the pipeline starts.
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Returns the confidence score threshold for detected objects.
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the non-maximum suppression (IoU) threshold. Candidates whose overlap with a kept detection is at most the threshold (inclusive) survive
     * suppression.
     *
     * @param threshold Non-maximum suppression threshold
     * @note Configures startup behavior. Send SCRFDParserConfig to inputConfig after the pipeline starts.
     */
    void setIouThreshold(float threshold);

    /**
     * Returns the non-maximum suppression (IoU) threshold.
     */
    float getIouThreshold() const;

    /**
     * Sets the maximum number of detections to keep.
     *
     * @param maxDetections Maximum number of detections to keep
     * @note Configures startup behavior. Send SCRFDParserConfig to inputConfig after the pipeline starts.
     */
    void setMaxDetections(int maxDetections);

    /**
     * Returns the maximum number of detections to keep.
     */
    int getMaxDetections() const;

    /**
     * Sets the model input image size the anchor centers are computed against and the emitted coordinates are normalized by. Defaults to (640, 640).
     *
     * Configuring from a full NNArchive derives it from the model input's declared shape and layout; the most recent configuration wins.
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
     * Sets the feature strides of the FPN. One score_{stride}, bbox_{stride} and kps_{stride} layer triple is parsed per stride. Defaults to (8, 16, 32).
     *
     * @param featStrideFpn Feature strides, every stride must be greater than 0
     */
    void setFeatStrideFPN(const std::vector<std::int64_t>& featStrideFpn);

    /**
     * Returns the feature strides of the FPN.
     */
    std::vector<std::int64_t> getFeatStrideFPN() const;

    /**
     * Sets the number of anchors per feature map position. Defaults to 2.
     *
     * @param numAnchors Number of anchors; values of 1 or less yield one anchor per position, mirroring the source behavior
     */
    void setNumAnchors(std::int64_t numAnchors);

    /**
     * Returns the number of anchors per feature map position.
     */
    std::int64_t getNumAnchors() const;

    /**
     * Sets the label names for the detected objects. The first label name is assigned to every detection (all detections carry label 0). When empty, no label
     * name is assigned. Defaults to ("Face").
     *
     * @param labelNames List of label names
     */
    void setLabelNames(const std::vector<std::string>& labelNames);

    /**
     * Returns the label names for the detected objects.
     */
    std::vector<std::string> getLabelNames() const;

    /**
     * Select whether the node runs on the host or device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Returns true when this node runs on the host.
     *
     * Host-only pipelines always run the node on the host.
     */
    bool runOnHost() const override;

    void run() override;

   private:
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
