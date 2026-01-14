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
    /**
     * Construct a FeatureTracker node with properties.
     */
    FeatureTracker(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    std::shared_ptr<FeatureTrackerConfig> initialConfig = std::make_shared<FeatureTrackerConfig>();

    /**
     * Input FeatureTrackerConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::FeatureTrackerConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, true}};

    /**
     * Outputs TrackedFeatures message that carries tracked features results.
     */
    Output outputFeatures{*this, {"outputFeatures", DEFAULT_GROUP, {{{DatatypeEnum::TrackedFeatures, false}}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{*this, {"passthroughInputImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

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
