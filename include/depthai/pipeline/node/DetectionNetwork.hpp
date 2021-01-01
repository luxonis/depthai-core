#pragma once

#include <depthai/pipeline/Node.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/pb/properties/DetectionNetworkProperties.hpp>

namespace dai {
namespace node {

class DetectionNetwork : public Node {
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    OpenVINO::Version networkOpenvinoVersion;

   protected:
    DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    dai::DetectionNetworkProperties properties;

   public:
    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, false}}};

    void setStreamName(const std::string& name);
    void setConfidenceThreshold(float thresh);
    void setNNBlobPath(const std::string& path);
    void setNumPoolFrames(int numFrames);
};

class MobileNetDetectionNetwork : public DetectionNetwork {
   public:
    MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
};

class YoloDetectionNetwork : public DetectionNetwork {
   public:
    YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    void setNumClasses(const int numClasses);
    void setCoordinateSize(const int coordinates);
    void setAnchors(std::vector<float> anchors);
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    void setIouThreshold(float thresh);
};

}  // namespace node
}  // namespace dai
