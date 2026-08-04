#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "depthai/beta/datatype/Keypoints.hpp"
#include "depthai/beta/properties/SuperAnimalParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief SuperAnimalParser node. Parses the heatmap output of the SuperAnimal landmark neural network into a dai::beta::Keypoints message.
 *
 * The parser consumes a single output tensor. When the incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer
 * name must be configured explicitly or through an NNArchive head. The tensor is read in its stored order and must be a 4D tensor of shape
 * (batch, height, width, numKeypoints) with a batch size of exactly 1. The number of keypoints and the heatmap size are derived from the tensor shape; the
 * configured number of keypoints is informational only and does not affect the decoding. Per keypoint, the keypoint is the position of the maximum heatmap
 * value with a 0.5-pixel center offset, mapped to input-image pixels and normalized by the configured scale factor without clipping, and the keypoint's score
 * is the heatmap value at that position, which must lie in [0, 1]. Keypoints with a score strictly below the score threshold are dropped and the skeleton
 * edges are remapped to the kept keypoints.
 *
 */
class SuperAnimalParser : public DeviceNodeCRTP<DeviceNode, SuperAnimalParser, SuperAnimalParserProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "SuperAnimalParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with heatmaps data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Keypoints message with the detected animal keypoints.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Keypoints, false}}}}};

    /**
     * @brief Build SuperAnimalParser node. Links the supplied output to this node's input and configures the parser from the model's SuperAnimalParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<SuperAnimalParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build SuperAnimalParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<SuperAnimalParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one SuperAnimalParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a SuperAnimalParser head with exactly one output layer.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the model output layer to parse.
     *
     * When left empty, the parser selects the tensor automatically if the incoming NNData contains exactly one tensor and fails otherwise.
     *
     * @param outputLayerName Name of the output layer
     */
    void setOutputLayerName(const std::string& outputLayerName);

    /**
     * Returns the name of the model output layer to parse.
     */
    std::string getOutputLayerName() const;

    /**
     * Sets the scale factor the keypoint coordinates are scaled and normalized by, typically the model input size.
     *
     * @param scaleFactor Scale factor, must be greater than 0
     */
    void setScaleFactor(float scaleFactor);

    /**
     * Returns the scale factor the keypoint coordinates are scaled and normalized by.
     */
    float getScaleFactor() const;

    /**
     * Sets the number of keypoints the model detects.
     *
     * Informational only: the decoding derives the number of keypoints from the heatmap tensor's last dimension.
     *
     * @param nKeypoints Number of keypoints, must be greater than 0
     */
    void setNumKeypoints(std::int64_t nKeypoints);

    /**
     * Returns the number of keypoints the model detects.
     */
    std::int64_t getNumKeypoints() const;

    /**
     * Sets the confidence score threshold for detected keypoints. Keypoints with a score strictly below the threshold are dropped.
     *
     * @param threshold Confidence score threshold, must be between 0 and 1
     */
    void setScoreThreshold(float threshold);

    /**
     * Returns the confidence score threshold for detected keypoints.
     */
    float getScoreThreshold() const;

    /**
     * Sets the label names for the keypoints, indexed by keypoint index.
     *
     * @param labelNames Vector of label names
     */
    void setLabelNames(const std::vector<std::string>& labelNames);

    /**
     * Returns the label names for the keypoints.
     */
    std::vector<std::string> getLabelNames() const;

    /**
     * Sets the skeleton edges as pairs of keypoint indices used for visualizing the skeleton.
     *
     * Example: {{0, 1}, {1, 2}, {2, 3}, {3, 0}} connects keypoint 0 to keypoint 1, keypoint 1 to keypoint 2, etc.
     *
     * @param edges Vector of keypoint index pairs
     */
    void setEdges(const std::vector<Edge>& edges);

    /**
     * Returns the skeleton edges as pairs of keypoint indices.
     */
    std::vector<Edge> getEdges() const;

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
};

}  // namespace node
}  // namespace beta
}  // namespace dai
