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
class AprilTag : public DeviceNodeCRTP<DeviceNode, AprilTag, AprilTagProperties> {
   public:
    constexpr static const char* NAME = "AprilTag";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

   public:
    AprilTag() = default;
    AprilTag(std::unique_ptr<Properties> props);

    std::shared_ptr<AprilTag> build() {
        return std::static_pointer_cast<AprilTag>(shared_from_this());
    }
    /**
     * Initial config to use when calculating spatial location data.
     */
    AprilTagConfig initialConfig;

    /**
     * Input AprilTagConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {.name = "inputConfig", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::AprilTagConfig, false}}}};

    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, {.name = "inputImage", .blocking = false, .queueSize = 4, .types = {{DatatypeEnum::ImgFrame, false}}}};

    /**
     * Outputs AprilTags message that carries spatial location results.
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::AprilTags, false}}}};

    /**
     * Outputs AprilTagConfig message that contains current configuration.
     */
    Output outConfig{*this, {.name = "outConfig", .types = {{DatatypeEnum::AprilTagConfig, false}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputImage{*this, {.name = "passthroughInputImage", .types = {{DatatypeEnum::ImgFrame, false}}}};
    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    void setWaitForConfigInput(bool wait);
};

}  // namespace node
}  // namespace dai
