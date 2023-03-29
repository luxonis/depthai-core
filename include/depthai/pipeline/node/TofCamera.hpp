#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/TofCameraProperties.hpp>

#include "depthai/pipeline/datatype/TofCameraConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief TofCamera node.
 * Performs feature tracking and reidentification using motion estimation between 2 consecutive frames.
 */
class TofCamera : public NodeCRTP<DeviceNode, TofCamera, TofCameraProperties> {
   public:
    constexpr static const char* NAME = "TofCamera";
    using NodeCRTP::NodeCRTP;

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawTofCameraConfig> rawConfig;

   public:
    TofCamera();
    TofCamera(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    TofCameraConfig initialConfig;

    /**
     * Input TofCameraConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{true, *this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::TofCameraConfig, false}}};
    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputRaw{true, *this, "inputRaw", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs TrackedFeatures message that carries tracked features results.
     */
    Output depth{true, *this, "depth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputRaw{true, *this, "passthroughInputRaw", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

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

    void setRawSize(int width, int height);
    int getRawWidth() const;
    int getRawHeight() const;

    void setDepthSize(int width, int height);
    int getDepthWidth() const;
    int getDepthHeight() const;
    TofSensorModel getTofModel();
    void setTofModel(TofSensorModel model);
};

}  // namespace node
}  // namespace dai
