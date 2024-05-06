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
    std::shared_ptr<SpatialLocationCalculator> build() {
        return std::static_pointer_cast<SpatialLocationCalculator>(shared_from_this());
    }

   protected:
    Properties& getProperties();

   public:
    SpatialLocationCalculator() = default;
    SpatialLocationCalculator(std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    SpatialLocationCalculatorConfig initialConfig;

    /**
     * Input SpatialLocationCalculatorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {.name = "inputConfig", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {.name = "inputDepth", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::ImgFrame, false}}}};

    /**
     * Outputs SpatialLocationCalculatorData message that carries spatial location results.
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::SpatialLocationCalculatorData, false}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {.name = "passthroughDepth", .types = {{DatatypeEnum::ImgFrame, false}}}};
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
