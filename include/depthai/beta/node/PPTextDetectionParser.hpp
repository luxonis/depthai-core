#pragma once

#include <memory>
#include <string>
#include <variant>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/PPTextDetectionParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief PPTextDetectionParser node. Parses the output of the PaddlePaddle OCR text detection model into a dai::ImgDetections message containing the rotated
 * bounding boxes and confidence scores of the detected text.
 *
 * The parser consumes a single probability-map output tensor of shape (1, 1, H, W) or (1, H, W, 1). The map is thresholded with the mask threshold into a
 * binary text mask, the mask is dilated and its contours become rotated-rectangle candidates; when more contours than the maximum number of detections
 * remain, the largest by area are kept. Rectangles smaller than 8 pixels on their smaller side are dropped, each candidate is scored with the mean probability
 * inside its (slightly shrunk) corner polygon, candidates scoring below the confidence threshold are dropped, and the kept rectangles are expanded by sqrt(2)
 * in both dimensions. The emitted bounding boxes are normalized to [0, 1] with angles in degrees rounded to whole numbers; the detections carry no labels.
 *
 */
class PPTextDetectionParser : public DeviceNodeCRTP<BetaNode, PPTextDetectionParser, PPTextDetectionParserProperties> {
   public:
    constexpr static const char* NAME = "PPTextDetectionParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with text detection probability map to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgDetections message with the rotated bounding boxes and confidence scores of the detected text.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * @brief Build PPTextDetectionParser node. Links the supplied output to this node's input and configures the parser from the model's
     * PPTextDetectionParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<PPTextDetectionParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build PPTextDetectionParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<PPTextDetectionParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one PPTextDetectionParser head; use setNNArchiveHead() to select a specific head
     * from a multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head configures the confidence threshold, the mask threshold and the maximum number of detections when
     * present in its metadata. The head's declared output names are not consumed: the parser resolves the single runtime output tensor by itself (or uses the
     * explicitly configured output layer name), mirroring the source parser; archives whose head output name differs from the model's declared output name
     * therefore parse correctly.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the model output layer holding the text probability map. When empty (the default), the layer is resolved automatically from
     * single-tensor NN results; multi-tensor results require an explicit name.
     *
     * @param outputLayerName Name of the output layer
     */
    void setOutputLayerName(const std::string& outputLayerName);

    /**
     * Returns the name of the model output layer holding the text probability map.
     */
    std::string getOutputLayerName() const;

    /**
     * Sets the confidence score threshold for the detected text bounding boxes. Candidates with a score strictly below the threshold are dropped.
     *
     * @param threshold Confidence score threshold
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Returns the confidence score threshold for the detected text bounding boxes.
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the mask threshold for creating the binary text mask from the model output probabilities. Probabilities strictly above the threshold belong to the
     * mask.
     *
     * @param maskThreshold Mask threshold
     */
    void setMaskThreshold(float maskThreshold);

    /**
     * Returns the mask threshold for creating the binary text mask from the model output probabilities.
     */
    float getMaskThreshold() const;

    /**
     * Sets the maximum number of candidate bounding boxes. When more candidate contours are found, only the largest by area are kept.
     *
     * @param maxDetections Maximum number of candidate bounding boxes
     */
    void setMaxDetections(int maxDetections);

    /**
     * Returns the maximum number of candidate bounding boxes.
     */
    int getMaxDetections() const;

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
