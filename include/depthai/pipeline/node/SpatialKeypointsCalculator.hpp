#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/SpatialKeypointsCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/Keypoints.hpp"

namespace dai {
namespace node {

/**
 * @brief SpatialKeypointsCalculator node. Calculates spatial location data on a set of ROIs on depth map.
 */
class SpatialKeypointsCalculator : public DeviceNodeCRTP<DeviceNode, SpatialKeypointsCalculator, SpatialKeypointsCalculatorProperties> {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "SpatialKeypointsCalculator";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    SpatialKeypointsCalculator() = default;
    SpatialKeypointsCalculator(std::unique_ptr<Properties> props);

    /**
     * Input SpatialKeypointsCalculatorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputKeypoints{*this, {"inputKeypoints", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Keypoints, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SpatialImgDetections, false}}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    SpatialKeypointsCalculator& setDepthThresholds(uint32_t lowerThreshold = 0, uint32_t upperThreshold = 65535);
    SpatialKeypointsCalculator& setCalculationAlgorithm(
        SpatialKeypointsCalculatorAlgorithm calculationAlgorithm = SpatialKeypointsCalculatorAlgorithm::AVERAGE);
    SpatialKeypointsCalculator& setMeasurementModel(SpatialKeypointsMeasurementModes measurementModel = SpatialKeypointsMeasurementModes::DETAILED);
    SpatialKeypointsCalculator& setStepSize(int32_t stepSize = SpatialKeypointsCalculatorProperties::AUTO);
};

}  // namespace node
}  // namespace dai
