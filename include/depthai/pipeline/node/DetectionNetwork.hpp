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
    const Properties& getPropertiesRef() const override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    /**
     * Outputs ImgDetections message that carries parsed detection results.
     * Overrides NeuralNetwork 'out' with ImgDetections output message type.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgDetections, false}}};

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
