#pragma once

#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/DetectionNetworkDepthProperties.hpp>

namespace dai {
namespace node {
class DetectionNetworkDepth : public DetectionNetwork {
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    dai::NeuralNetworkProperties& getPropertiesRef() override;

   protected:
    DetectionNetworkDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    dai::DetectionNetworkDepthProperties properties;

   public:
    /**
     * Input message with data to be infered upon
     * Default queue is blocking with size 5
     */
    using DetectionNetwork::input;
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, true, 5, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgDetections, false}}};
    Output passthroughRoi{*this, "passthroughRoi", Output::Type::MSender, {{DatatypeEnum::DepthCalculatorConfig, false}}};

    using DetectionNetwork::passthrough;

    void setConfidenceThreshold(float thresh);
    void setBoundingBoxScaleFactor(float factor);
    void setDepthLowerThreshold(uint32_t lowerThreshold);
    void setDepthUpperThreshold(uint32_t upperThreshold);
};

class MobileNetDetectionNetworkDepth : public DetectionNetworkDepth {
   public:
    MobileNetDetectionNetworkDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
};

class YoloDetectionNetworkDepth : public DetectionNetworkDepth {
   public:
    YoloDetectionNetworkDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    void setNumClasses(const int numClasses);
    void setCoordinateSize(const int coordinates);
    void setAnchors(std::vector<float> anchors);
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    void setIouThreshold(float thresh);
};

}  // namespace node
}  // namespace dai
