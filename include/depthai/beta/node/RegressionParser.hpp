#pragma once

#include <memory>
#include <string>
#include <variant>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/datatype/Predictions.hpp"
#include "depthai/beta/properties/RegressionParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief RegressionParser node. Parses the output of a model with regression output (e.g. age-gender) into a dai::beta::Predictions message with the
 * predicted value(s) in the order the model emitted them.
 *
 * The parser consumes a single output tensor. When the incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer
 * name must be configured explicitly or through an NNArchive head. The tensor is dequantized and all its singleton dimensions are squeezed; the remaining
 * values become the predictions, so any tensor with at most one non-singleton dimension is accepted regardless of rank (for example (1, 1, 1, 3), (1, 1) or
 * (1,)) and an empty tensor yields a message with no predictions. A tensor with more than one non-singleton dimension after squeezing is rejected.
 */
class RegressionParser : public DeviceNodeCRTP<BetaNode, RegressionParser, RegressionParserProperties> {
   public:
    constexpr static const char* NAME = "RegressionParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with regression data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Predictions message with the predicted value(s).
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Predictions, false}}}}};

    /**
     * @brief Build RegressionParser node. Links the supplied output to this node's input and configures the parser from the model's RegressionParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<RegressionParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build RegressionParser node with the specific head from an NNArchive. Useful when the model has multiple regression heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<RegressionParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one RegressionParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a RegressionParser head with exactly one output layer.
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
