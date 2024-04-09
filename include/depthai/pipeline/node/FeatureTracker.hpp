#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/FeatureTrackerProperties.hpp>

#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief FeatureTracker node.
 * Performs feature tracking and reidentification using motion estimation between 2 consecutive frames.
 */
class FeatureTracker : public NodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties> {
   public:
    constexpr static const char* NAME = "FeatureTracker";
    using NodeCRTP::NodeCRTP;

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawFeatureTrackerConfig> rawConfig;

   public:
    FeatureTracker();
    FeatureTracker(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    FeatureTrackerConfig initialConfig;

    /**
     * Input FeatureTrackerConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{true, *this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::FeatureTrackerConfig, false}}};
    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{true, *this, "inputImage", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs TrackedFeatures message that carries tracked features results.
     */
    Output outputFeatures{true, *this, "outputFeatures", Output::Type::MSender, {{DatatypeEnum::TrackedFeatures, false}}};

    /**
     * Outputs descriptor message that carries tracked features descriptors.
     */
    Output outputDescriptors{true, *this, "outputDescriptors", Output::Type::MSender, {{DatatypeEnum::Buffer, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{true, *this, "passthroughInputImage", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

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

    /**
     * Specify allocated hardware resources for feature tracking.
     * 2 shaves/memory slices are required for optical flow, 1 for corner detection only.
     * @param numShaves Number of shaves. Maximum 2.
     * @param numMemorySlices Number of memory slices. Maximum 2.
     */
    void setHardwareResources(int numShaves, int numMemorySlices);
};

}  // namespace node
}  // namespace dai
