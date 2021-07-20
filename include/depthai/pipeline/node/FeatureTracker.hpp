#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/FeatureTrackerProperties.hpp>

#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief FeatureTracker node. Calculates spatial location data on a set of ROIs on depth map.
 */
class FeatureTracker : public Node {
   public:
    using Properties = dai::FeatureTrackerProperties;

   private:
    std::string getName() const override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawFeatureTrackerConfig> rawConfig;
    Properties properties;

   public:
    FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Initial config to use for feature tracking.
     */
    FeatureTrackerConfig initialConfig;

    /**
     * Input FeatureTrackerConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::FeatureTrackerConfig, false}}};
    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, "inputImage", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs FeatureTrackerData message that carries tracked features results.
     */
    Output outputFeatures{*this, "outputFeatures", Output::Type::MSender, {{DatatypeEnum::FeatureTrackerData, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{*this, "passthroughInputImage", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    void setWaitForConfigInput(bool wait);
};

}  // namespace node
}  // namespace dai
