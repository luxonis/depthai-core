#pragma once

#include <depthai/modelzoo/Zoo.hpp>
#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/nn_archive/NNArchive.hpp>
#include <depthai/nn_archive/NNArchiveVersionedConfig.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <depthai/properties/YoloSegmentationParserProperties.hpp>
#include <optional>

namespace dai {
namespace node {

/**
 * @brief YoloSegmentationParser node. Parses detection results from different neural networks and is being used internally by MobileNetDetectionNetwork and
 * YoloDetectionNetwork.
 */
class YoloSegmentationParser : public DeviceNodeCRTP<DeviceNode, YoloSegmentationParser, YoloSegmentationParserProperties> {
   public:
    constexpr static const char* NAME = "YoloSegmentationParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * @brief Build YoloSegmentationParser node. Connect output to this node's input. Also call setNNArchive() with provided NNArchive.
     * @param nnInput: Output to link
     * @param nnArchive: Neural network archive
     */
    std::shared_ptr<YoloSegmentationParser> build(Node::Output& nnInput, const NNArchive& nnArchive);
    /**
     * Input NN results with detection data to parse
     * Default queue is blocking with size 5
     */
    Input input{*this, {"input", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NNData, true}}}, true}};

    /**
     * Outputs image frame with detected edges
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    void setNNArchive(const NNArchive& nnArchive);

   private:
    void setNNArchiveOther(const NNArchive& nnArchive);
    void setConfig(const dai::NNArchiveVersionedConfig& config);

    std::optional<NNArchive> mArchive;

    std::optional<NNArchiveVersionedConfig> archiveConfig;
};

}  // namespace node
}  // namespace dai
