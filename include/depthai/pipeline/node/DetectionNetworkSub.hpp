#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/node/DetectionParser.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

namespace dai {
namespace node {

/**
 * @brief DetectionNetworkSub, base for different network specializations
 */
class DetectionNetworkSub : public NeuralNetwork {
   public:
    constexpr static const char* NAME = "DetectionNetworkSub";
    void build();
    DetectionNetworkSub()
        : out{detectionParser->out},
          outNetwork{neuralNetwork->out},
          input{neuralNetwork->input},
          passthrough{neuralNetwork->passthrough} {};
    // No public constructor, only a factory function.
    [[nodiscard]] static std::shared_ptr<DetectionNetworkSub> create() {
        auto n = std::make_shared<DetectionNetworkSub>();
        n->build();
        return n;
    }

   public:
    Subnode<NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};
    Subnode<DetectionParser> detectionParser{*this, "detectionParser"};
    /**
     * Outputs ImgDetections message that carries parsed detection results.
     * Overrides NeuralNetwork 'out' with ImgDetections output message type.
     */
    Output& out;

    /**
     * Outputs unparsed inference results.
     */
    Output& outNetwork;

    /**
     * Input message with data to be inferred upon
     * Default queue is blocking with size 5
     */
    Input& input;

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output& passthrough;

    /**
     * Specifies confidence threshold at which to filter the rest of the detections.
     * @param thresh Detection confidence must be greater than specified threshold to be added to the list
     */
    void setConfidenceThreshold(float thresh);

    /**
     * Retrieves threshold at which to filter the rest of the detections.
     * @returns Detection confidence
     */
    float getConfidenceThreshold() const;
};

/**
 * @brief MobileNetDetectionNetworkSub node. Parses MobileNet results
 */
class MobileNetDetectionNetworkSub : public DetectionNetworkSub {
   public:
    [[nodiscard]] static std::shared_ptr<MobileNetDetectionNetworkSub> create() {
        auto n = std::make_shared<MobileNetDetectionNetworkSub>();
        n->build();
        return n;
    }
    void build();
};

/**
 * @brief YoloDetectionNetworkSub node. Parses Yolo results
 */
class YoloDetectionNetworkSub : public DetectionNetworkSub {
   public:
    void build();
    [[nodiscard]] static std::shared_ptr<YoloDetectionNetworkSub> create() {
        auto n = std::make_shared<YoloDetectionNetworkSub>();
        n->build();
        return n;
    }

    /// Set num classes
    void setNumClasses(int numClasses);
    /// Set coordianate size
    void setCoordinateSize(int coordinates);
    /// Set anchors
    void setAnchors(std::vector<float> anchors);
    /// Set anchor masks
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    /// Set Iou threshold
    void setIouThreshold(float thresh);

    /// Get num classes
    int getNumClasses() const;
    /// Get coordianate size
    int getCoordinateSize() const;
    /// Get anchors
    std::vector<float> getAnchors() const;
    /// Get anchor masks
    std::map<std::string, std::vector<int>> getAnchorMasks() const;
    /// Get Iou threshold
    float getIouThreshold() const;
};

}  // namespace node
}  // namespace dai
