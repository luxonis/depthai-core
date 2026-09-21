#pragma once

#include <memory>
#include <string>
#include <variant>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/EmbeddingsParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief EmbeddingsParser node. Validates the raw output of an embeddings neural network model head and forwards it unchanged as a dai::NNData message.
 *
 * The parser expects a single output tensor carrying the embedding vector. When the output layer name is left unconfigured, every incoming NNData must contain
 * exactly one tensor; otherwise the message is rejected. The message itself is forwarded without modification, so all tensors, sequence number, timestamps,
 * and image transformation metadata are preserved.
 */
class EmbeddingsParser : public DeviceNodeCRTP<BetaNode, EmbeddingsParser, EmbeddingsParserProperties> {
   public:
    constexpr static const char* NAME = "EmbeddingsParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with embeddings data to validate and forward.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs the unchanged NNData message containing the embeddings output layer.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::NNData, false}}}}};

    /**
     * @brief Build EmbeddingsParser node. Links the supplied output to this node's input and configures the parser from the model's embeddings head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<EmbeddingsParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build EmbeddingsParser node with the specific head from an NNArchive. Useful when the model has multiple embeddings heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<EmbeddingsParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one EmbeddingsParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an EmbeddingsParser head with exactly one output layer.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the model output layer carrying the embeddings.
     *
     * When left empty, the parser requires the incoming NNData to contain exactly one tensor and fails otherwise.
     *
     * @param outputLayerName Name of the output layer
     */
    void setOutputLayerName(const std::string& outputLayerName);

    /**
     * Returns the name of the model output layer carrying the embeddings.
     */
    std::string getOutputLayerName() const;

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
