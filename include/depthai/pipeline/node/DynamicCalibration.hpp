#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include <DynamicCalibration.hpp>

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

    float qualityCheck = 0.0f;
    // We could have a map here to perform the calibration on an arbitrary number of inputs for example: (left, right, rgb).
    // /**
    //  * A map of inputs
    //  */
    // InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Input left{*this, {"left", DEFAULT_GROUP, false, 1}};
    Input right{*this, {"right", DEFAULT_GROUP, false, 1}};

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
    float getCalibQuality() const;
    auto createDCLCameraCalibration(const std::vector<std::vector<float>> cameraMatrix, const std::vector<float> distortionCoefficients, const std::vector<std::vector<float>> rotationMatrix, const std::vector<float> translationVector);
    void pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket, int widthDefault = 1280, int heightDefault = 800);
    void run() override;

    private:
        std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R);
        std::unique_ptr<dcl::DynamicCalibration> dynCalibImpl;
        dcl::mxid_t deviceName;
        dcl::socket_t socketA;
        dcl::socket_t socketB;
        const int initialSkipFrames = 500;
        const int processEveryNFrames = 5;
};

}  // namespace node
}  // namespace dai
