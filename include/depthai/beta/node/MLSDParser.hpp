#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <variant>

#include "depthai/beta/datatype/Lines.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief MLSDParser node. Parses the output of the M-LSD line segment detection model into a dai::beta::Lines message with the detected lines and their
 * confidence scores, ordered by descending score.
 *
 * The parser consumes two output tensors that must be configured before the pipeline starts, either explicitly or through an NNArchive head: the tpMap tensor,
 * read in NCHW orientation as a 4D tensor of shape (batch, channels, height, width) whose channels 1 to 4 hold the line displacement maps of the first batch
 * entry, and the heat tensor, flattened to one score per (height, width) grid position. The topK highest-scoring grid positions are decoded into candidate
 * lines and kept when their score and length are strictly above the score and distance thresholds. Ties between equal heat scores are ordered following
 * numpy's portable argpartition/argsort semantics. The emitted line coordinates are normalized by the model input size, which defaults to 512x512 (the input
 * size of all known M-LSD models, hard-coded by the source parser); building from a full NNArchive derives it from the model input's declared shape and
 * layout (NHWC or NCHW), and setInputSize() overrides it.
 *
 * @note This node runs on the host only.
 */
class MLSDParser : public NodeCRTP<dai::node::ThreadedHostNode, MLSDParser> {
   public:
    constexpr static const char* NAME = "MLSDParser";
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with line detection data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Lines message with the detected lines and confidence scores.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Lines, false}}}}};

    /**
     * @brief Build MLSDParser node. Links the supplied output to this node's input and configures the parser from the model's MLSDParser head and the model
     * input's declared shape and layout.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<MLSDParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build MLSDParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     * @note A head carries no model input metadata, so the input size keeps its current value (512x512 by default); use setInputSize() for models with a
     * different input size.
     */
    std::shared_ptr<MLSDParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one MLSDParser head and exactly one model input; use setNNArchiveHead() to select a
     * specific head from a multi-head archive. The input size is derived from the model input's declared shape and layout (NHWC or NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an MLSDParser head with exactly two output layers; the layer whose name contains "tpMap" is
     * used as the tpMap layer and the layer whose name contains "heat" as the heat layer.
     *
     * @param head: NNArchive head to set
     * @note A head carries no model input metadata, so the input size keeps its current value (512x512 by default); use setInputSize() for models with a
     * different input size.
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the output layer containing the tpMap tensor.
     *
     * Must be configured before the pipeline starts, either explicitly or through an NNArchive head.
     *
     * @param outputLayerTPMap Name of the output layer containing the tpMap tensor
     */
    void setOutputLayerTPMap(const std::string& outputLayerTPMap);

    /**
     * Returns the name of the output layer containing the tpMap tensor, or an empty string when not configured.
     */
    std::string getOutputLayerTPMap() const;

    /**
     * Sets the name of the output layer containing the heat tensor.
     *
     * Must be configured before the pipeline starts, either explicitly or through an NNArchive head.
     *
     * @param outputLayerHeat Name of the output layer containing the heat tensor
     */
    void setOutputLayerHeat(const std::string& outputLayerHeat);

    /**
     * Returns the name of the output layer containing the heat tensor, or an empty string when not configured.
     */
    std::string getOutputLayerHeat() const;

    /**
     * Sets the number of top candidates to keep.
     *
     * The number of candidates is capped at the heat map size when decoding.
     *
     * @param topK Number of top candidates to keep, must be positive
     */
    void setTopK(int topK);

    /**
     * Returns the number of top candidates to keep.
     */
    int getTopK() const;

    /**
     * Sets the confidence score threshold for detected lines. Candidates with a heat score strictly above the threshold are kept.
     *
     * @param scoreThreshold Confidence score threshold
     */
    void setScoreThreshold(float scoreThreshold);

    /**
     * Returns the confidence score threshold for detected lines.
     */
    float getScoreThreshold() const;

    /**
     * Sets the distance threshold for detected lines. Candidates whose length in heat map grid units is strictly above the threshold are kept.
     *
     * @param distanceThreshold Distance threshold
     */
    void setDistanceThreshold(float distanceThreshold);

    /**
     * Returns the distance threshold for detected lines.
     */
    float getDistanceThreshold() const;

    /**
     * Sets the model input image size the emitted line coordinates are normalized by, x coordinates by the width and y coordinates by the height.
     *
     * Defaults to 512x512, the input size of all known M-LSD models. Configuring from a full NNArchive derives it from the model input's declared shape and
     * layout; the most recent configuration wins.
     *
     * @param width Input image width, must be greater than 0
     * @param height Input image height, must be greater than 0
     */
    void setInputSize(std::uint32_t width, std::uint32_t height);

    /**
     * Returns the model input image size as (width, height).
     */
    std::pair<std::uint32_t, std::uint32_t> getInputSize() const;

    void run() override;

   private:
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    std::string outputLayerTPMap;
    std::string outputLayerHeat;
    int topK = 200;
    float scoreThreshold = 0.10f;
    float distanceThreshold = 20.0f;
    std::pair<std::uint32_t, std::uint32_t> inputSize{512, 512};
};

}  // namespace node
}  // namespace beta
}  // namespace dai
