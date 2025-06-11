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

   protected:
    Properties& getProperties();

   public:
    ToF() = default;
    ToF(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    std::shared_ptr<ToFConfig> initialConfig = std::make_shared<ToFConfig>();

    /**
     * Input ToFConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this,
                      {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ToFConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output amplitude{*this, {"amplitude", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output intensity{*this, {"intensity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output phase{*this, {"phase", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Build with a specific board socket
     */
    std::shared_ptr<ToF> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO, float fps = 30);

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

   private:
    bool isBuilt = false;
    uint32_t maxWidth = 0;
    uint32_t maxHeight = 0;
};

}  // namespace node
}  // namespace dai
