#pragma once

#include <memory>
#include <string>
#include <variant>

#include "depthai/beta/datatype/Map2D.hpp"
#include "depthai/beta/properties/MapOutputParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief MapOutputParser node. Parses the output of models that produce map outputs, such as depth maps (e.g. DepthAnything), density maps (e.g. DM-Count),
 * heat maps, and similar, into a dai::beta::Map2D message.
 *
 * The parser consumes a single output tensor. When the incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer
 * name must be configured explicitly or through an NNArchive head. The tensor is read in its stored order; leading dimensions of 1 are squeezed and the
 * tensor must then be a 2D HW map, or a 3D HWN map with a singleton trailing dimension that is squeezed as well. All map dimensions are derived from the
 * runtime tensor descriptor.
 *
 * When min-max scaling is enabled, the map values are scaled to the [0, 1] range; a constant map is left unchanged.
 *
 */
class MapOutputParser : public DeviceNodeCRTP<DeviceNode, MapOutputParser, MapOutputParserProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "MapOutputParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with map tensor data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Map2D message with the parsed 2D map, e.g. a depth or density map.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Map2D, false}}}}};

    /**
     * @brief Build MapOutputParser node. Links the supplied output to this node's input and configures the parser from the model's MapOutputParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<MapOutputParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build MapOutputParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<MapOutputParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one MapOutputParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a MapOutputParser head with exactly one output layer.
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
     * Sets the flag indicating whether the map is scaled to the [0, 1] range.
     *
     * When true, the map values are min-max scaled to [0, 1]; a constant map is left unchanged. Defaults to false.
     *
     * @param minMaxScaling True to scale the map to the [0, 1] range, defaults to true
     */
    void setMinMaxScaling(bool minMaxScaling = true);

    /**
     * Returns the flag indicating whether the map is scaled to the [0, 1] range.
     */
    bool getMinMaxScaling() const;

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
