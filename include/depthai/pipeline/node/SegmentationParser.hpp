#pragma once

#include <depthai/modelzoo/Zoo.hpp>
#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <depthai/nn_archive/NNArchive.hpp>
#include <depthai/nn_archive/NNArchiveVersionedConfig.hpp>
#include <depthai/properties/SegmentationParserProperties.hpp>
#include <memory>
#include <optional>

namespace dai {
namespace node {

class SegmentationParser : public DeviceNodeCRTP<DeviceNode, SegmentationParser, SegmentationParserProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    std::optional<NNArchive> mArchive;
    std::optional<NNArchiveVersionedConfig> archiveConfig;

   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "SegmentationParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    ~SegmentationParser() override;
    SegmentationParser() = default;
    SegmentationParser(std::unique_ptr<Properties> props)
        : DeviceNodeCRTP(std::move(props)), initialConfig(std::make_shared<SegmentationParserConfig>(properties.parserConfig)), inConfig(initialConfig) {}

    /**
     * Initial config to use when parsing segmentation masks.
     */
    std::shared_ptr<SegmentationParserConfig> initialConfig = std::make_shared<SegmentationParserConfig>();

    std::shared_ptr<SegmentationParserConfig> inConfig;

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
    std::shared_ptr<SegmentationParser> build(Node::Output& nnInput, const NNArchive& nnArchive);

    /**
     * @brief Build SegmentationParser node with the specific head from NNArchive. Useful when model outputs multiple segmentation heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<SegmentationParser> build(Node::Output& nnInput, const std::optional<dai::nn_archive::v1::Head>& head);

    /**
     * @brief Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default number of shaves.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

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
};

}  // namespace node
}  // namespace dai
