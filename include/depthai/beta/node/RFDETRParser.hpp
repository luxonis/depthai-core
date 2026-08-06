#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/RFDETRParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief RFDETRParser node. Parses the output of RF-DETR object detection models
 * (https://github.com/roboflow/rf-detr) into a dai::ImgDetections message containing the bounding boxes, labels, confidence scores and, in segmentation mode,
 * an instance segmentation mask, everything normalized to [0, 1].
 *
 * The parser consumes 2 output tensors for detection (boxes, class logits) or 3 for instance segmentation (boxes, class logits, mask logits), in that order.
 * When no output layer names are configured, all layer names of the incoming NNData are used in their reported order. The boxes tensor squeezes to (N, 4) with
 * normalized (xCenter, yCenter, width, height) boxes, the logits tensor is (1, N, C) and the mask logits tensor squeezes to (N, maskHeight, maskWidth). Class
 * probabilities are the sigmoid of the logits; per query the maximum probability is the score and its class the label. Detections are ordered by descending
 * score, truncated to the maximum number of detections and kept when their score is strictly greater than the confidence threshold.
 *
 * In segmentation mode at most 255 instances fit into the mask, so the truncation is additionally capped at 255. Each detection's mask logits are passed
 * through a sigmoid, cropped to its bounding box, binarized with the mask confidence threshold and resized to the model input size with nearest-neighbor
 * interpolation; the pixels not claimed by an earlier (higher-scoring) detection receive the detection's index, with 255 marking background.
 *
 */
class RFDETRParser : public DeviceNodeCRTP<BetaNode, RFDETRParser, RFDETRParserProperties> {
   public:
    constexpr static const char* NAME = "RFDETRParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with RF-DETR detection data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgDetections message with the bounding boxes, labels and confidence scores of the detected objects and, in segmentation mode, the instance
     * segmentation mask.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * @brief Build RFDETRParser node. Links the supplied output to this node's input and configures the parser from the model's RFDETRParser head and the
     * model input's declared shape and layout.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<RFDETRParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build RFDETRParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     * @note A head carries no model input metadata, so the input size keeps its current value; segmentation mode requires it to be configured with
     * setInputSize() when it is not set yet.
     */
    std::shared_ptr<RFDETRParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one RFDETRParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive. The input size is derived from the first model input's declared shape and layout (NHWC or NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an RFDETRParser head with 2 output layers (boxes, class logits) for detection or 3 (boxes,
     * class logits, mask logits) for segmentation.
     *
     * @param head: NNArchive head to set
     * @note A head carries no model input metadata, so the input size keeps its current value; segmentation mode requires it to be configured with
     * setInputSize() when it is not set yet.
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the confidence score threshold for detected objects. Detections with a score strictly greater than the threshold are kept. Defaults to 0.5.
     *
     * @param threshold Confidence score threshold, must be between 0 and 1
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Returns the confidence score threshold for detected objects.
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the maximum number of detections to keep, applied to the detections ordered by descending score. In segmentation mode the applied limit is
     * additionally capped at 255, the maximum number of instances the segmentation mask can encode. Defaults to 300.
     *
     * @param maxDetections Maximum number of detections to keep, must be greater than 0
     */
    void setMaxDetections(int maxDetections);

    /**
     * Returns the maximum number of detections to keep.
     */
    int getMaxDetections() const;

    /**
     * Sets the label names for the detected objects, indexed by the class label. A detection whose label is out of range receives the name
     * "class_<label>". When empty, no label names are assigned. Defaults to empty.
     *
     * @param labelNames List of label names
     */
    void setLabelNames(const std::vector<std::string>& labelNames);

    /**
     * Returns the label names for the detected objects.
     */
    std::vector<std::string> getLabelNames() const;

    /**
     * Sets the mask confidence threshold used to binarize instance segmentation masks in segmentation mode. Mask pixels with a sigmoid probability strictly
     * greater than the threshold belong to the instance. Defaults to 0.5.
     *
     * @param maskConfidence Mask confidence threshold, must be between 0 and 1
     */
    void setMaskConfidence(float maskConfidence);

    /**
     * Returns the mask confidence threshold.
     */
    float getMaskConfidence() const;

    /**
     * Sets the names of the model output layers, positionally: the boxes layer, the class logits layer and, in segmentation mode, the mask logits layer. Must
     * hold 2 or 3 names.
     *
     * When left empty, all layer names of the incoming NNData are used in their reported order.
     *
     * @param outputLayerNames Names of the output layers
     */
    void setOutputLayerNames(const std::vector<std::string>& outputLayerNames);

    /**
     * Returns the names of the model output layers.
     */
    std::vector<std::string> getOutputLayerNames() const;

    /**
     * Sets the model input image size the segmentation mask is emitted at. Unset by default; segmentation mode requires it to be configured from an NNArchive
     * or with this setter before the parser processes messages. Detection mode does not use it.
     *
     * Configuring from a full NNArchive derives it from the first model input's declared shape and layout; the most recent configuration wins.
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
