#pragma once

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "depthai/beta/datatype/Classifications.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief ClassificationParser node. Parses the raw output of a classification neural network into a dai::beta::Classifications message with class names and
 * scores sorted in descending order of score.
 *
 * The parser consumes a single output tensor. When the incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer
 * name must be configured explicitly or through an NNArchive head. Raw scores are dequantized and flattened; when the model output is not already softmaxed,
 * the parser applies softmax to convert the scores to probabilities.
 *
 * @note This node runs on the host only.
 */
class ClassificationParser : public NodeCRTP<dai::node::ThreadedHostNode, ClassificationParser> {
   public:
    constexpr static const char* NAME = "ClassificationParser";
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with classification data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Classifications message with classes and scores sorted in descending order of score.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Classifications, false}}}}};

    /**
     * @brief Build ClassificationParser node. Links the supplied output to this node's input and configures the parser from the model's classification head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<ClassificationParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build ClassificationParser node with the specific head from an NNArchive. Useful when the model has multiple classification heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<ClassificationParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one ClassificationParser head; use setNNArchiveHead() to select a specific head
     * from a multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a ClassificationParser head with exactly one output layer.
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
     * Sets the class names to link with the classification scores.
     *
     * The class names are expected to be in the same order as the neural network's output. The number of class names must match the number of scores produced
     * by the model.
     *
     * @param classes Vector of class names
     */
    void setClasses(const std::vector<std::string>& classes);

    /**
     * Returns the class names to link with the classification scores.
     */
    std::vector<std::string> getClasses() const;

    /**
     * Sets whether the model output is already softmaxed.
     *
     * When false, the parser applies softmax to convert the raw scores to probabilities.
     *
     * @param isSoftmax True when the model output is already softmaxed
     */
    void setSoftmax(bool isSoftmax);

    /**
     * Returns whether the model output is treated as already softmaxed.
     */
    bool getSoftmax() const;

    void run() override;

   private:
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    std::string outputLayerName;
    std::vector<std::string> classes;
    std::int64_t nClasses = 0;
    bool isSoftmax = true;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
