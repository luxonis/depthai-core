#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/SpatialLocationCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief SpatialLocationCalculator node. Calculates spatial location data on a set of ROIs on depth map.
 */
class SpatialLocationCalculator : public DeviceNodeCRTP<DeviceNode, SpatialLocationCalculator, SpatialLocationCalculatorProperties> {
   public:
    constexpr static const char* NAME = "SpatialLocationCalculator";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

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
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs SpatialLocationCalculatorData message that carries spatial location results.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SpatialLocationCalculatorData, false}}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] void setWaitForConfigInput(bool wait);

    /**
     * @see setWaitForConfigInput
     * @returns True if wait for inputConfig message, false otherwise
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] bool getWaitForConfigInput() const;
};

}  // namespace node
}  // namespace dai
