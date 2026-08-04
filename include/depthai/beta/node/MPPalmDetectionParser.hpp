#pragma once

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "depthai/beta/properties/MPPalmDetectionParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief MPPalmDetectionParser node. Parses the output of the MediaPipe palm detection model into a dai::ImgDetections message containing the rotated bounding
 * boxes, labels and confidence scores of the detected hands. The decoding is based on https://github.com/geaxgx/depthai_hand_tracker (MIT License).
 *
 * The parser consumes two output tensors and identifies them by their last dimension: the tensor with the larger last dimension holds the raw bounding boxes
 * and is reshaped to (numAnchors, 18) rows of bounding box center/size plus 7 palm keypoint coordinate pairs; the tensor with the smaller last dimension holds
 * the raw scores and is flattened to (numAnchors,). The scores are passed through a sigmoid and filtered with the confidence threshold, the kept rows are
 * decoded against the model's SSD anchors generated from the configured scale (the model input size), converted to rectangles rotated to align the wrist to
 * middle-finger direction with the rectangle's y-axis and expanded to squares, and non-maximum suppression keeps at most the configured maximum number of
 * detections. The emitted bounding boxes are normalized to [0, 1].
 *
 */
class MPPalmDetectionParser : public DeviceNodeCRTP<DeviceNode, MPPalmDetectionParser, MPPalmDetectionParserProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "MPPalmDetectionParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with palm detection data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgDetections message with the rotated bounding boxes, labels and confidence scores of the detected hands.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * @brief Build MPPalmDetectionParser node. Links the supplied output to this node's input and configures the parser from the model's
     * MPPalmDetectionParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<MPPalmDetectionParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build MPPalmDetectionParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<MPPalmDetectionParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one MPPalmDetectionParser head; use setNNArchiveHead() to select a specific head
     * from a multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an MPPalmDetectionParser head with exactly two output layers.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the names of the model output layers relevant to the parser. Exactly two layer names are required.
     *
     * @param outputLayerNames Names of the output layers
     */
    void setOutputLayerNames(const std::vector<std::string>& outputLayerNames);

    /**
     * Returns the names of the model output layers relevant to the parser.
     */
    std::vector<std::string> getOutputLayerNames() const;

    /**
     * Sets the confidence score threshold for detected hands. Detections with a sigmoid score strictly above the threshold are kept.
     *
     * @param threshold Confidence score threshold
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Returns the confidence score threshold for detected hands.
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the non-maximum suppression (IoU) threshold.
     *
     * @param threshold Non-maximum suppression threshold
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
     */
    void setMaxDetections(int maxDetections);

    /**
     * Returns the maximum number of detections to keep.
     */
    int getMaxDetections() const;

    /**
     * Sets the scale of the model input image in pixels (e.g. 192 for a 192x192 model). The SSD anchors used for decoding are generated from the scale; a
     * scale that does not match the model input size fails decoding with an anchor count mismatch.
     *
     * @param scale Scale of the input image
     */
    void setScale(int scale);

    /**
     * Returns the scale of the model input image in pixels.
     */
    int getScale() const;

    /**
     * Sets the label names for the detected hands. The first label name is assigned to every detection (all detections carry label 0). When empty, no label
     * name is assigned.
     *
     * @param labelNames List of label names
     */
    void setLabelNames(const std::vector<std::string>& labelNames);

    /**
     * Returns the label names for the detected hands.
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
