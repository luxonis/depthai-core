#pragma once

#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/DetectionNetworkProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief DetectionNetwork, base for different network specializations
 */
class DetectionNetwork : public NeuralNetwork {
   public:
    using Properties = dai::DetectionNetworkProperties;

    std::string getName() const override;

   protected:
    Properties properties;

    DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    Properties& getPropertiesRef() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    /**
     * Input message with data to be infered upon
     * Default queue is blocking with size 5
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 5, {{DatatypeEnum::Buffer, true}}};

    /**
     * Outputs ImgDetections message that carries parsed detection results.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgDetections, false}}};

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Specifies confidence threshold at which to filter the rest of the detections.
     * @param thresh Detection confidence must be greater than specified threshold to be added to the list
     */
    void setConfidenceThreshold(float thresh);
};

/**
 * @brief MobileNetDetectionNetwork node. Parses MobileNet results
 */
class MobileNetDetectionNetwork : public DetectionNetwork {
   protected:
    std::shared_ptr<Node> clone() override;

   public:
    MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
};

/**
 * @brief YoloDetectionNetwork node. Parses Yolo results
 */
class YoloDetectionNetwork : public DetectionNetwork {
   protected:
    std::shared_ptr<Node> clone() override;

   public:
    YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /// Set num classes
    void setNumClasses(const int numClasses);
    /// Set coordianate size
    void setCoordinateSize(const int coordinates);
    /// Set anchors
    void setAnchors(std::vector<float> anchors);
    /// Set anchor masks
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    /// Set Iou threshold
    void setIouThreshold(float thresh);
};

}  // namespace node
}  // namespace dai
