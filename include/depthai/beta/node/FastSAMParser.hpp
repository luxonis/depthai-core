#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/FastSAMParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief FastSAMParser node. Parses the output of the FastSAM segmentation model
 * (https://github.com/CASIA-IVA-Lab/FastSAM) into a dai::SegmentationMask message where each pixel holds the index of the instance it belongs to and 255
 * marks background.
 *
 * The parser consumes the model's YOLO detection outputs (NCHW tensors of shape (1, numClasses + 5, gridH, gridW), sorted by layer name and decoded
 * anchorless with strides 8/16/32), the per-head mask-coefficient outputs (NCHW tensors of shape (1, numPrototypes, gridH, gridW), sorted by layer name) and
 * the prototype masks output (NCHW tensor of shape (1, numPrototypes, protoH, protoW)). The model input size is derived from the first (stride-8) YOLO
 * output's grid times 8; the number of prototypes from the protos tensor's channel count. Boxes pass confidence filtering and non-maximum suppression, boxes
 * within 20 pixels of the image border are snapped to it, and a box overlapping the full image with IoU > 0.9 is replaced by the full-image box. Each kept
 * detection's mask is combined from the prototypes, resized to the model input size with nearest-neighbor interpolation, cropped to its box and binarized
 * with the mask confidence threshold.
 *
 * The prompt selects the emitted instances: "everything" keeps all detections (later, lower-confidence instances overwrite earlier ones on overlapping
 * pixels), "bbox" keeps the single mask with the highest IoU against the prompt bounding box, and "point" combines the masks containing the prompt point
 * (added for point label 1, subtracted for 0). With no detections, a fully-background mask is emitted.
 *
 */
class FastSAMParser : public DeviceNodeCRTP<BetaNode, FastSAMParser, FastSAMParserProperties> {
   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "FastSAMParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    FastSAMParser() = default;
    FastSAMParser(std::unique_ptr<Properties> props);

    /** Configuration used until a message is received on inputConfig. */
    std::shared_ptr<FastSAMParserConfig> initialConfig = std::make_shared<FastSAMParserConfig>();

    /**
     * Runtime parser configuration. When synchronized, one configuration is consumed per frame;
     * otherwise all queued configurations are drained and the newest valid one is used.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::FastSAMParserConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input NN results with FastSAM data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs SegmentationMask message with the resulting segmentation masks given the prompt.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SegmentationMask, false}}}}};

    /**
     * @brief Build FastSAMParser node. Links the supplied output to this node's input and configures the parser from the model's FastSAMParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<FastSAMParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build FastSAMParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<FastSAMParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one FastSAMParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head's output layer names containing "_yolo" configure the YOLO output layers and those containing
     * "_masks" the mask output layers (each only when at least one matches). The confidence threshold, number of classes, NMS threshold, mask confidence,
     * prompt, points, point label and bounding box are read from the head metadata when present.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the confidence score threshold for detected objects. Detections whose score is strictly greater than the threshold are kept. Defaults to 0.5.
     *
     * @param threshold Confidence score threshold, must be between 0 and 1
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Returns the confidence score threshold for detected objects.
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the number of classes in the model. The YOLO output tensors must have numClasses + 5 channels. Defaults to 1.
     *
     * @param numClasses Number of classes, must be greater than 0
     */
    void setNumClasses(std::int32_t numClasses);

    /**
     * Returns the number of classes in the model.
     */
    std::int32_t getNumClasses() const;

    /**
     * Sets the non-maximum suppression overlap threshold. Boxes whose overlap with a kept box is strictly greater than the threshold are suppressed.
     * Defaults to 0.5.
     *
     * @param iouThreshold Overlap threshold, must be between 0 and 1
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setIouThreshold(float iouThreshold);

    /**
     * Returns the non-maximum suppression overlap threshold.
     */
    float getIouThreshold() const;

    /**
     * Sets the mask confidence threshold used to binarize instance masks. Mask pixels with a sigmoid probability strictly greater than the threshold belong
     * to the instance. Defaults to 0.5.
     *
     * @param maskConfidence Mask confidence threshold, must be between 0 and 1
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setMaskConfidence(float maskConfidence);

    /**
     * Returns the mask confidence threshold.
     */
    float getMaskConfidence() const;

    /**
     * Sets the prompt type: "everything" emits every detected instance, "bbox" the single instance mask with the highest IoU against the prompt bounding
     * box (see setBoundingBox()), and "point" the combination of the instance masks containing the prompt point (see setPoints() and setPointLabel()).
     * Defaults to "everything".
     *
     * @param prompt Prompt type, one of "everything", "bbox" or "point"
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setPrompt(const std::string& prompt);

    /**
     * Returns the prompt type.
     */
    std::string getPrompt() const;

    /**
     * Sets the prompt point as (x, y) in model-input pixels, used by the "point" prompt. Unset by default; the "point" prompt requires it.
     *
     * @param x Point x coordinate
     * @param y Point y coordinate
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setPoints(std::int32_t x, std::int32_t y);

    /**
     * Returns the prompt point as (x, y), or std::nullopt when it is not set.
     */
    std::optional<std::pair<std::int32_t, std::int32_t>> getPoints() const;

    /**
     * Sets the prompt point label, used by the "point" prompt: 1 adds the instance masks containing the point, 0 subtracts them. Unset by default; the
     * "point" prompt requires it.
     *
     * @param pointLabel Point label
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setPointLabel(std::int32_t pointLabel);

    /**
     * Returns the prompt point label, or std::nullopt when it is not set.
     */
    std::optional<std::int32_t> getPointLabel() const;

    /**
     * Sets the prompt bounding box as (x1, y1, x2, y2) in model-input pixels, used by the "bbox" prompt. Unset by default; the "bbox" prompt requires it and
     * its x2 and y2 coordinates must not be 0.
     *
     * @param bbox Bounding box as (x1, y1, x2, y2)
     * @note Configures startup behavior. Send FastSAMParserConfig to inputConfig after the pipeline starts.
     */
    void setBoundingBox(const std::array<std::int32_t, 4>& bbox);

    /**
     * Returns the prompt bounding box as (x1, y1, x2, y2), or std::nullopt when it is not set.
     */
    std::optional<std::array<std::int32_t, 4>> getBoundingBox() const;

    /**
     * Sets the names of the model's YOLO output layers. The layers are processed sorted by name, so the stride-8 head must come first in sort order.
     * Defaults to ["output1_yolov8", "output2_yolov8", "output3_yolov8"].
     *
     * @param yoloOutputs Names of the YOLO output layers
     */
    void setYoloOutputs(const std::vector<std::string>& yoloOutputs);

    /**
     * Returns the names of the model's YOLO output layers.
     */
    std::vector<std::string> getYoloOutputs() const;

    /**
     * Sets the names of the model's mask-coefficient output layers. Only names containing "mask" are used, sorted by name and index-aligned with the sorted
     * YOLO output layers; when empty, all layer names of the incoming NNData containing "mask" are used. Defaults to ["output1_masks", "output2_masks",
     * "output3_masks"].
     *
     * @param maskOutputs Names of the mask output layers
     */
    void setMaskOutputs(const std::vector<std::string>& maskOutputs);

    /**
     * Returns the names of the model's mask-coefficient output layers.
     */
    std::vector<std::string> getMaskOutputs() const;

    /**
     * Sets the name of the model's prototype-masks output layer; when empty, "protos_output" is used. Defaults to "protos_output".
     *
     * @param protosOutput Name of the protos output layer
     */
    void setProtosOutput(const std::string& protosOutput);

    /**
     * Returns the name of the model's prototype-masks output layer.
     */
    std::string getProtosOutput() const;

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
