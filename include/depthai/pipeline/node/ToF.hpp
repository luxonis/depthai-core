#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/ToFProperties.hpp>

#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief ToF node.
 * Performs feature tracking and reidentification using motion estimation between 2 consecutive frames.
 */
class ToF : public DeviceNodeCRTP<DeviceNode, ToF, ToFProperties> {
   public:
    constexpr static const char* NAME = "ToF";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    std::shared_ptr<ToF> build() {
        return std::static_pointer_cast<ToF>(shared_from_this());
    }

   protected:
    Properties& getProperties();

   public:
    ToF() = default;
    ToF(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    ToFConfig initialConfig;

    /**
     * Input ToFConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this,
                      {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ToFConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputRaw{*this, {"inputRaw", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    // Note on API limitation:
    // TODO(before mainline) - API not supported on RVC3
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputRaw{*this, {"passthroughInputRaw", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    // Note on API limitation:
    // TODO(before mainline) - API not supported on RVC3
    Output amplitude{*this, {"amplitude", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output error{*this, {"error", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output intensity{*this, {"intensity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
};

}  // namespace node
}  // namespace dai
