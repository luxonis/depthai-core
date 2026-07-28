#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief YuNetParser node. Parses the output of the YuNet face detection model into a dai::ImgDetections message containing the bounding boxes, labels,
 * confidence scores and 5 facial keypoints per detected face, everything normalized to [0, 1]. The decoding is based on
 * https://github.com/Kazuhito00/YuNet-ONNX-TFLite-Sample (Apache License 2.0).
 *
 * The parser consumes three output tensors: a loc tensor with 14 values per anchor (2 bounding box center offsets, 2 bounding box size values and 5 keypoint
 * coordinate offset pairs), a conf tensor with 2 values per anchor (non-face and face scores) and an iou tensor with 1 value per anchor. When a layer name is
 * not configured, it is auto-detected from the incoming NNData as the single layer name starting with "loc", "conf" or "iou" respectively. The candidate
 * scores are sqrt(conf face score * iou score clipped to [0, 1]); candidates with a score strictly greater than the confidence threshold are decoded against
 * the YuNet anchors generated from the input size, suppressed with cv2.dnn.NMSBoxes-semantics non-maximum suppression (IoU threshold, maximum number of
 * detections as the top-k limit) and emitted in descending score order. Keypoint coordinates are truncated to whole pixels before normalization, mirroring the
 * source parser. The anchors are cached across messages and refreshed when the input size changes.
 *
 * @note This node runs on the host only.
 */
class YuNetParser : public NodeCRTP<dai::node::ThreadedHostNode, YuNetParser> {
   public:
    constexpr static const char* NAME = "YuNetParser";
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with YuNet detection data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgDetections message with the bounding boxes, labels, confidence scores and keypoints of the detected faces.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * @brief Build YuNetParser node. Links the supplied output to this node's input and configures the parser from the model's YuNetParser head and the model
     * input's declared shape and layout.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<YuNetParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build YuNetParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     * @note A head carries no model input metadata, so the input size keeps its current value; configure it with setInputSize() when it is not set yet.
     */
    std::shared_ptr<YuNetParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one YuNetParser head and exactly one model input; use setNNArchiveHead() to select
     * a specific head from a multi-head archive. The input size is derived from the model input's declared shape and layout (NHWC or NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a YuNetParser head; every head output layer name must contain "loc", "conf" or "iou" and is
     * routed to the matching output layer name setting.
     *
     * @param head: NNArchive head to set
     * @note A head carries no model input metadata, so the input size keeps its current value; configure it with setInputSize() when it is not set yet.
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the output layer containing the location predictions. When left empty, the name is auto-detected from the incoming NNData as the
     * single layer name starting with "loc".
     *
     * @param locOutputLayerName Output layer name for the loc tensor
     */
    void setOutputLayerLoc(const std::string& locOutputLayerName);

    /**
     * Returns the name of the output layer containing the location predictions.
     */
    std::string getOutputLayerLoc() const;

    /**
     * Sets the name of the output layer containing the confidence predictions. When left empty, the name is auto-detected from the incoming NNData as the
     * single layer name starting with "conf".
     *
     * @param confOutputLayerName Output layer name for the conf tensor
     */
    void setOutputLayerConf(const std::string& confOutputLayerName);

    /**
     * Returns the name of the output layer containing the confidence predictions.
     */
    std::string getOutputLayerConf() const;

    /**
     * Sets the name of the output layer containing the IoU predictions. When left empty, the name is auto-detected from the incoming NNData as the single
     * layer name starting with "iou".
     *
     * @param iouOutputLayerName Output layer name for the IoU tensor
     */
    void setOutputLayerIou(const std::string& iouOutputLayerName);

    /**
     * Returns the name of the output layer containing the IoU predictions.
     */
    std::string getOutputLayerIou() const;

    /**
     * Sets the confidence score threshold for detected faces. Detections with a score strictly greater than the threshold are kept.
     *
     * @param threshold Confidence score threshold
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Returns the confidence score threshold for detected faces.
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the non-maximum suppression (IoU) threshold. Candidates whose overlap with a kept detection is at most the threshold (inclusive) survive
     * suppression.
     *
     * @param threshold Non-maximum suppression threshold
     */
    void setIouThreshold(float threshold);

    /**
     * Returns the non-maximum suppression (IoU) threshold.
     */
    float getIouThreshold() const;

    /**
     * Sets the maximum number of detections to keep, applied as the non-maximum suppression top-k limit (no limit when 0 or negative).
     *
     * @param maxDetections Maximum number of detections to keep
     */
    void setMaxDetections(int maxDetections);

    /**
     * Returns the maximum number of detections to keep.
     */
    int getMaxDetections() const;

    /**
     * Sets the model input image size the anchors are computed against and the emitted coordinates are normalized by. Unset by default; it must be configured
     * from an NNArchive or with this setter before the parser processes messages.
     *
     * Configuring from a full NNArchive derives it from the model input's declared shape and layout; the most recent configuration wins.
     *
     * @param width Input image width, must be greater than 0
     * @param height Input image height, must be greater than 0
     */
    void setInputSize(std::uint32_t width, std::uint32_t height);

    /**
     * Returns the model input image size as (width, height), or std::nullopt when it is not set.
     */
    std::optional<std::pair<std::uint32_t, std::uint32_t>> getInputSize() const;

    /**
     * Sets the label names for the detected faces. The first label name is assigned to every detection (all detections carry label 0). When empty, no label
     * name is assigned. Defaults to ("Face").
     *
     * @param labelNames List of label names
     */
    void setLabelNames(const std::vector<std::string>& labelNames);

    /**
     * Returns the label names for the detected faces.
     */
    std::vector<std::string> getLabelNames() const;

    void run() override;

   private:
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    std::string locOutputLayerName;
    std::string confOutputLayerName;
    std::string iouOutputLayerName;
    float confidenceThreshold = 0.8f;
    float iouThreshold = 0.3f;
    int maxDetections = 5000;
    std::optional<std::pair<std::uint32_t, std::uint32_t>> inputSize;
    std::vector<std::string> labelNames{"Face"};
};

}  // namespace node
}  // namespace beta
}  // namespace dai
