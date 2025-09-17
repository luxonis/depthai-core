#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/SpatialDetectionCalculatorProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SpatialDetectionCalculator node. Calculates spatial location data on a set of ROIs on depth map.
 */
class SpatialDetectionCalculator : public DeviceNodeCRTP<DeviceNode, SpatialDetectionCalculator, SpatialDetectionCalculatorProperties> {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "SpatialDetectionCalculator";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    SpatialDetectionCalculator() = default;
    SpatialDetectionCalculator(std::unique_ptr<Properties> props);

    /**
     * Input SpatialDetectionCalculatorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputDetections{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgDetections, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs SpatialDetectionCalculatorData message that carries spatial location results.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SpatialImgDetections, false}}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    SpatialDetectionCalculator& setRunOnHost(bool runOnHost = true);

    SpatialDetectionCalculator& setDepthThresholds(uint32_t lowerThreshold = 0, uint32_t upperThreshold = 65535);
    SpatialDetectionCalculator& setCalculationAlgorithm(SpatialDetectionCalculatorAlgorithm calculationAlgorithm = SpatialDetectionCalculatorAlgorithm::MEDIAN);
    SpatialDetectionCalculator& setMeasurementModel(SpatialDetectionsMeasurementModes measurementModel = SpatialDetectionsMeasurementModes::DETAILED);
    SpatialDetectionCalculator& setStepSize(int32_t stepSize = SpatialDetectionCalculatorProperties::AUTO);

    bool runOnHost() const override;

    void run() override;
};

}  // namespace node
}  // namespace dai
