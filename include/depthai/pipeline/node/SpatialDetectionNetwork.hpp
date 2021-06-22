#pragma once

#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/SpatialDetectionNetworkProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SpatialDetectionNetwork node. Runs a neural inference on input image and calculates spatial location data.
 */
class SpatialDetectionNetwork : public DetectionNetwork {
   public:
    using Properties = dai::SpatialDetectionNetworkProperties;

    std::string getName() const override;

   protected:
    Properties properties;
    virtual Properties& getPropertiesRef() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    SpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Input message with data to be infered upon
     * Default queue is blocking with size 5
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 5, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object
     * Default queue is non-blocking with size 4
     */
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgDetections message that carries parsed detection results.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SpatialImgDetections, false}}};

    /**
     * Outputs mapping of detected bounding boxes relative to depth map
     *
     * Suitable for when displaying remapped bounding boxes on depth frame
     */
    Output boundingBoxMapping{*this, "boundingBoxMapping", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}};

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough message for depth frame on which the spatial location calculation was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specifies scale factor for detected bounding boxes.
     * @param scaleFactor Scale factor must be in the interval (0,1].
     */
    void setBoundingBoxScaleFactor(float scaleFactor);

    /**
     * Specifies lower threshold in milimeters for depth values which will used to calculate spatial data
     * @param lowerThreshold LowerThreshold must be in the interval [0,upperThreshold] and less than upperThreshold.
     */
    void setDepthLowerThreshold(uint32_t lowerThreshold);

    /**
     * Specifies upper threshold in milimeters for depth values which will used to calculate spatial data
     * @param upperThreshold UpperThreshold must be in the interval (lowerThreshold,65535].
     */
    void setDepthUpperThreshold(uint32_t upperThreshold);
};

/**
 * MobileNetSpatialDetectionNetwork node. Mobilenet-SSD based network with spatial location data.
 */
class MobileNetSpatialDetectionNetwork : public SpatialDetectionNetwork {
   protected:
    std::shared_ptr<Node> clone() override;

   public:
    MobileNetSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
};

/**
 * YoloSpatialDetectionNetwork node. (tiny)Yolov3/v4 based network with spatial location data.
 */
class YoloSpatialDetectionNetwork : public SpatialDetectionNetwork {
   protected:
    std::shared_ptr<Node> clone() override;

   public:
    YoloSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
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
