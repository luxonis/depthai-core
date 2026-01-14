#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/AprilTagProperties.hpp>

#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief AprilTag node.
 */
class AprilTag : public DeviceNodeCRTP<DeviceNode, AprilTag, AprilTagProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "AprilTag";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties() override;

   public:
    AprilTag() = default;
    /**
     * Construct an AprilTag node with properties.
     */
    AprilTag(std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    std::shared_ptr<AprilTagConfig> initialConfig = std::make_shared<AprilTagConfig>();

    /**
     * Input AprilTagConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::AprilTagConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs AprilTags message that carries spatial location results.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::AprilTags, false}}}}};

    /**
     * Outputs AprilTagConfig message that contains current configuration.
     */
    Output outConfig{*this, {"outConfig", DEFAULT_GROUP, {{{DatatypeEnum::AprilTagConfig, false}}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{*this, {"passthroughInputImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    void setWaitForConfigInput(bool wait);

    /**
     * @brief Get whether or not wait until configuration message arrives to inputConfig Input.
     */
    bool getWaitForConfigInput() const;

    /**
     * Set number of threads to use for AprilTag detection.
     * @param numThreads Number of threads to use.
     */
    void setNumThreads(int numThreads);

    /**
     * Get number of threads to use for AprilTag detection.
     * @return Number of threads to use.
     */
    int getNumThreads() const;

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
