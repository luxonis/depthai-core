#pragma once

#include <cstdint>
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
 * @brief ClassificationSequenceParser node. Parses the raw output of a classification sequence neural network into a dai::beta::Classifications message with
 * class names and scores ordered by their position in the sequence.
 *
 * The model predicts the classes multiple times and returns a list of predicted classes, where each item corresponds to the relative step in the sequence. In
 * addition to time series classification, this parser can also be used for text recognition models where words can be interpreted as a sequence of characters
 * (classes).
 *
 * The parser consumes a single output tensor of shape (sequenceLength, nClasses), (1, sequenceLength, nClasses) or (sequenceLength, nClasses, 1). When the
 * incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer name must be configured explicitly or through an
 * NNArchive head. Raw scores are dequantized; when the model output is not already softmaxed, the parser applies softmax along each sequence step to convert
 * the scores to probabilities.
 *
 * @note This node runs on the host only.
 */
class ClassificationSequenceParser : public NodeCRTP<dai::node::ThreadedHostNode, ClassificationSequenceParser> {
   public:
    constexpr static const char* NAME = "ClassificationSequenceParser";
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with classification sequence data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Classifications message with classes and scores ordered by their position in the sequence.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Classifications, false}}}}};

    /**
     * @brief Build ClassificationSequenceParser node. Links the supplied output to this node's input and configures the parser from the model's classification
     * sequence head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<ClassificationSequenceParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build ClassificationSequenceParser node with the specific head from an NNArchive. Useful when the model has multiple classification sequence
     * heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<ClassificationSequenceParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one ClassificationSequenceParser head; use setNNArchiveHead() to select a specific
     * head from a multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a ClassificationSequenceParser head with exactly one output layer.
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
     * Sets the class names to link with the per-step classification scores.
     *
     * The class names are expected to be in the same order as the neural network's output. The number of class names must match the number of scores produced
     * by the model at each sequence step.
     *
     * @param classes Vector of class names
     */
    void setClasses(const std::vector<std::string>& classes);

    /**
     * Returns the class names to link with the per-step classification scores.
     */
    std::vector<std::string> getClasses() const;

    /**
     * Sets whether the model output is already softmaxed.
     *
     * When false, the parser applies softmax along each sequence step to convert the raw scores to probabilities.
     *
     * @param isSoftmax True when the model output is already softmaxed
     */
    void setSoftmax(bool isSoftmax);

    /**
     * Returns whether the model output is treated as already softmaxed.
     */
    bool getSoftmax() const;

    /**
     * Sets the class indexes to ignore during classification sequence generation (e.g. background class, blank space).
     *
     * Sequence steps whose most probable class index is listed here are dropped from the output. Every index must be within [0, nClasses - 1].
     *
     * @param ignoredIndexes Vector of class indexes to ignore
     */
    void setIgnoredIndexes(const std::vector<std::int32_t>& ignoredIndexes);

    /**
     * Returns the class indexes ignored during classification sequence generation.
     */
    std::vector<std::int32_t> getIgnoredIndexes() const;

    /**
     * Sets whether consecutive duplicate classes are removed from the sequence.
     *
     * Only consecutive duplicates are removed; repeated classes separated by other classes are kept.
     *
     * @param removeDuplicates True to remove consecutive duplicates from the sequence
     */
    void setRemoveDuplicates(bool removeDuplicates);

    /**
     * Returns whether consecutive duplicate classes are removed from the sequence.
     */
    bool getRemoveDuplicates() const;

    /**
     * Sets whether the remaining classes are concatenated. Used mostly for text processing.
     *
     * When true and more than one class remains: when all remaining class names are at most one character long, they are joined and split on whitespace into
     * words with a per-word mean score; otherwise all class names are joined into a single string with a " " separator and one mean score.
     *
     * @param concatenateClasses True to concatenate the remaining classes
     */
    void setConcatenateClasses(bool concatenateClasses);

    /**
     * Returns whether the remaining classes are concatenated.
     */
    bool getConcatenateClasses() const;

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
    std::vector<std::int32_t> ignoredIndexes;
    bool removeDuplicates = false;
    bool concatenateClasses = false;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
