#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/beta/datatype/Clusters.hpp"
#include "depthai/beta/properties/LaneDetectionParserProperties.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief LaneDetectionParser node. Parses the output of an Ultra-Fast-Lane-Detection (UFLD) neural network, e.g. the CULane and TuSimple variants, into a
 * dai::beta::Clusters message with one cluster of normalized points per lane, including empty clusters for lanes without enough detected points.
 *
 * The parser consumes a single output tensor. When the incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer
 * name must be configured explicitly or through an NNArchive head. The tensor is read in its stored order and must be a 4D tensor of shape
 * (batch, gridingNum + 1, clsNumPerLane, numLanes); the first batch entry is decoded. The row anchors, griding number and number of points per lane must be
 * configured before the pipeline starts, either explicitly or through an NNArchive head. The input size must also be configured before the pipeline starts:
 * building from a full NNArchive derives it from the model input's declared shape and layout (NHWC or NCHW), while building from a specific head requires
 * setInputSize() because a head carries no model input metadata.
 *
 */
class LaneDetectionParser : public DeviceNodeCRTP<DeviceNode, LaneDetectionParser, LaneDetectionParserProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "LaneDetectionParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with lane detection data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Clusters message with the detected lanes represented as clusters of points.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Clusters, false}}}}};

    /**
     * @brief Build LaneDetectionParser node. Links the supplied output to this node's input and configures the parser from the model's LaneDetectionParser
     * head and the model input's declared shape and layout.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<LaneDetectionParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build LaneDetectionParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     * @note A head carries no model input metadata, so the input size must additionally be configured with setInputSize().
     */
    std::shared_ptr<LaneDetectionParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one LaneDetectionParser head and exactly one model input; use setNNArchiveHead()
     * to select a specific head from a multi-head archive. The input size is derived from the model input's declared shape and layout (NHWC or NCHW).
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be a LaneDetectionParser head with exactly one output layer.
     *
     * @param head: NNArchive head to set
     * @note A head carries no model input metadata, so the input size must additionally be configured with setInputSize().
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
     * Sets the row anchors, the image rows at which the model predicts lane positions.
     *
     * Must be configured before the pipeline starts, either explicitly or through an NNArchive head, and must contain at least as many entries as the number
     * of points per lane.
     *
     * @param rowAnchors Row anchors, must not be empty
     */
    void setRowAnchors(const std::vector<std::int64_t>& rowAnchors);

    /**
     * Returns the row anchors, or an empty vector when not configured.
     */
    std::vector<std::int64_t> getRowAnchors() const;

    /**
     * Sets the griding number, the number of column samples the model predicts lane positions over.
     *
     * Must be configured before the pipeline starts, either explicitly or through an NNArchive head.
     *
     * @param gridingNum Griding number, must be greater than 1
     */
    void setGridingNum(std::int64_t gridingNum);

    /**
     * Returns the griding number, or std::nullopt when not configured.
     */
    std::optional<std::int64_t> getGridingNum() const;

    /**
     * Sets the number of points per lane.
     *
     * Must be configured before the pipeline starts, either explicitly or through an NNArchive head.
     *
     * @param clsNumPerLane Number of points per lane, must be greater than 0
     */
    void setClsNumPerLane(std::int64_t clsNumPerLane);

    /**
     * Returns the number of points per lane, or std::nullopt when not configured.
     */
    std::optional<std::int64_t> getClsNumPerLane() const;

    /**
     * Sets the model input image size the emitted points are computed against and normalized by.
     *
     * Must be configured before the pipeline starts. Configuring from a full NNArchive derives it from the model input's declared shape and layout; the most
     * recent configuration wins.
     *
     * @param width Input image width, must be greater than 0
     * @param height Input image height, must be greater than 0
     */
    void setInputSize(std::uint32_t width, std::uint32_t height);

    /**
     * Returns the model input image size as (width, height), or std::nullopt when not configured.
     */
    std::optional<std::pair<std::uint32_t, std::uint32_t>> getInputSize() const;

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
