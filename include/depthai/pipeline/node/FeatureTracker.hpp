#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/FeatureTrackerProperties.hpp>

#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief FeatureTracker node.
 * Performs feature tracking and reidentification using motion estimation between 2 consecutive frames.
 */
class FeatureTracker : public DeviceNodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties> {
   public:
    constexpr static const char* NAME = "FeatureTracker";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

   public:
    FeatureTracker() = default;
    FeatureTracker(std::unique_ptr<Properties> props);

    std::shared_ptr<FeatureTracker> build() {
        return std::static_pointer_cast<FeatureTracker>(shared_from_this());
    }
    /**
     * Initial config to use for feature tracking.
     */
    FeatureTrackerConfig initialConfig;

    /**
     * Input FeatureTrackerConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {.name = "inputConfig", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::FeatureTrackerConfig, false}}}};

    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, {.name = "inputImage", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::ImgFrame, false}}, .waitForMessage = true}};

    /**
     * Outputs TrackedFeatures message that carries tracked features results.
     */
    Output outputFeatures{*this, {"outputFeatures", DEFAULT_GROUP, {{{DatatypeEnum::TrackedFeatures, false}}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{*this, {"passthroughInputImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
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
