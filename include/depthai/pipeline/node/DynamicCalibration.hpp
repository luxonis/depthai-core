#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Dynamic calibration node. Performs calibration check and dynamically calibrates the device
 */
class DynamicCalibration : public DeviceNodeCRTP<DeviceNode, DynamicCalibration, DynamicCalibrationProperties>, public HostRunnable {
   private:
    bool runOnHostVar = true;

   public:
    constexpr static const char* NAME = "DynamicCalibration";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    // We could have a map here to perform the calibration on an arbitrary number of inputs for example: (left, right, rgb).
    // /**
    //  * A map of inputs
    //  */
    // InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Input left{*this, {"left"}};
    Input right{*this, {"right"}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host on RVC2 and on device on RVC4.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    /**
     * Get the calibration quality (epipolar error)
     * @return Epipolar error in pixels
     */
    float getCalibrationQuality() const;

    void run() override;
};

}  // namespace node
}  // namespace dai
