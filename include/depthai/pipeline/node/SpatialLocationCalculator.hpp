#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/SpatialLocationCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief SpatialLocationCalculator node. Calculates the spatial locations of detected objects based on the input depth map. Spatial location calculations can
 * be additionally refined by using a segmentation mask. If keypoints are provided, the spatial location is calculated around each keypoint.
 */
class SpatialLocationCalculator : public DeviceNodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties> {
   public:
    constexpr static const char* NAME = "SpatialLocationCalculator";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties() override;

   public:
    SpatialLocationCalculator() = default;
    SpatialLocationCalculator(std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    std::shared_ptr<SpatialLocationCalculatorConfig> initialConfig = std::make_shared<SpatialLocationCalculatorConfig>();

    /**
     * Input SpatialLocationCalculatorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::SpatialLocationCalculatorConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input messages on which spatial location will be calculated.
     * Possible datatypes are ImgDetections or Keypoints.
     */
    Input input{*this, {"input", DEFAULT_GROUP, true, 1, {{{DatatypeEnum::ImgDetections, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs SpatialLocationCalculatorData message that carries spatial locations for each additional ROI that is specified in the config.
     * @warning Will be deprecated in future releases. Use spatialOutput instead.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SpatialLocationCalculatorData, false}}}}};

    /**
     * Outputs SpatialImgDetections or SpatialKeypoints message that carries spatial locations along with original input data.
     */
    Output spatialOutput{*this, {"spatialOutput", DEFAULT_GROUP, {{DatatypeEnum::SpatialImgDetections, false}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
};

}  // namespace node
}  // namespace dai
