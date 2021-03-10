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
class SpatialDetectionNetwork : public DetectionNetwork {
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;

   protected:
    SpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    dai::SpatialDetectionNetworkProperties properties;
    dai::SpatialDetectionNetworkProperties& getPropertiesRef() override;

   public:
    /**
     * Input message with data to be infered upon
     * Default queue is blocking with size 5
     */
    using DetectionNetwork::input;

    /**
     * nput message with depth data used to retrieve spatial information about detected object
     * Default queue is non-blocking with size 4
     */
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, false, 4, {{DatatypeEnum::Buffer, true}}};

    /**
     * Outputs ImgDetections message that carries parsed detection results.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SpatialImgDetections, false}}};

    /**
     * Passthrough message of depth calculator config for detected bounbing boxes
     *
     * Suitable for when displaying remapped bounding boxes on depth frame
     */
    Output passthroughRoi{*this, "passthroughRoi", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}};

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    using DetectionNetwork::passthrough;

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

class MobileNetSpatialDetectionNetwork : public SpatialDetectionNetwork {
   public:
    MobileNetSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
};

class YoloSpatialDetectionNetwork : public SpatialDetectionNetwork {
   public:
    YoloSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
};

}  // namespace node
}  // namespace dai
