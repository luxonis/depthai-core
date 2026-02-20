#pragma once

#include <depthai/modelzoo/Zoo.hpp>
#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"

// standard
#include <depthai/nn_archive/NNArchive.hpp>
#include <depthai/nn_archive/NNArchiveVersionedConfig.hpp>
#include <depthai/properties/SegmentationParserProperties.hpp>
#include <memory>
#include <optional>

namespace dai {
namespace node {

/**
 * @brief SegmentationParser node. Parses raw segmentation output from segmentation neural networks into a dai::SegmentationMask datatype.
 * The parser supports two output model types:
 * 1. Single-channel output where the model argmaxes the class probabilities internally and outputs a single channel mask with class indices.
 * 2. Multi-channel output where each channel corresponds to the probability map for a specific class. The parser will perform argmax across channels to
 * generate the final mask. The parser can be configured to treat the first class (index 0) as the background class, which will be ignored in the final
 * segmentation mask.
 *
 * @warning Only OAK4 supports running SegmentationParser on device. On other platforms, the node will automatically switch to host execution.
 */
class SegmentationParser : public DeviceNodeCRTP<DeviceNode, SegmentationParser, SegmentationParserProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "SegmentationParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    ~SegmentationParser() override;
    SegmentationParser() = default;
    SegmentationParser(std::unique_ptr<Properties> props) : DeviceNodeCRTP(std::move(props)) {}

    /**
     * Initial config to use when parsing segmentation masks.
     */
    std::shared_ptr<SegmentationParserConfig> initialConfig = std::make_shared<SegmentationParserConfig>();

    /**
     * Input NN results with segmentation data to parser
     */
    Input input{*this, {"input", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NNData, true}}}, true}};

    /**
     * Input SegmentationParserConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::SegmentationParserConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs segmentation mask
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SegmentationMask, false}}}}};

    /**
     * @brief Build SegmentationParser node
     * @param nnInput: Output to link
     * @param nnArchive: Neural network archive
     */
    std::shared_ptr<SegmentationParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build SegmentationParser node with the specific head from NNArchive. Useful when model outputs multiple segmentation heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<SegmentationParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * Sets the class labels associated with the segmentation mask.
     * The label at index $i$ in the `labels` vector corresponds to the value $i$ in the segmentation mask data array.
     * @param labels Vector of class labels
     */
    void setLabels(const std::vector<std::string>& labels);

    /**
     * Returns the class labels associated with the segmentation mask.
     */
    std::vector<std::string> getLabels() const;

    /**
     * Sets whether the first class (index 0) is considered the background class.
     * If true, the pixels classified as index 0 will be treated as background.
     * @param backgroundClass Boolean indicating if the first class is the background class
     *
     * @note Only applicable if the number of classes is greater than 1 and the output classes are not in a single layer (eg. classesInOneLayer = false).
     */
    void setBackgroundClass(bool backgroundClass);

    /**
     * Gets whether the first class (index 0) is considered the background class.
     */
    bool getBackgroundClass() const;

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

    void buildInternal() override;

   private:
    bool runOnHostVar = false;
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    void validateTensor(std::optional<TensorInfo>& info);
    std::optional<NNArchiveVersionedConfig> archiveConfig;
    std::shared_ptr<SegmentationParserConfig> inConfig;
    // TODO (aljazkonec1): common functions that are shared with NeuralNetwork, DetectionNetwork, etc. should be moved to a common base class
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

   protected:
    Properties& getProperties() override;
};

}  // namespace node
}  // namespace dai
