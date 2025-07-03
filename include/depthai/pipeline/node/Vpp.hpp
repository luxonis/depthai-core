#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

// shared
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/properties/VppProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief Vpp node. Apply Virtual Projection Pattern algorithm to stereo images based on disparity.
 */
class Vpp : public DeviceNodeCRTP<DeviceNode, Vpp, VppProperties> {
   public:
    constexpr static const char* NAME = "Vpp";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    ~Vpp() = default;

   protected:
    Properties& getProperties();
    Vpp() = default;
    Vpp(std::unique_ptr<Properties> props);

   public:
    /**
     * Initial config to use for VPP.
     */
    std::shared_ptr<VppConfig> initialConfig = std::make_shared<VppConfig>();

    /**
     * Input VppConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::VppConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for left ImgFrame
     */
    Input left{*this, {"left", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for right ImgFrame
     */
    Input right{*this, {"right", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for disparity ImgFrame (RAW16 or float32)
     */
    Input disparity{*this, {"disparity", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Output ImgFrame message that carries the processed left image with virtual projection pattern applied.
     */
    Output leftOut{*this, {"leftOut", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output ImgFrame message that carries the processed right image with virtual projection pattern applied.
     */
    Output rightOut{*this, {"rightOut", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
};

}  // namespace node
}  // namespace dai
