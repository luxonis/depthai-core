#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

// shared
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/VppConfig.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/VppProperties.hpp>

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

    Input syncInput{*this, {"inSync", DEFAULT_GROUP, false, 1, {{DatatypeEnum::MessageGroup, true}}}};
    // clang-format on

    void buildInternal() override;

    Subnode<node::Sync> sync{*this, "sync"};

    InputMap& inputs = sync->inputs;

    std::string leftInputName = "left";

    std::string rightInputName = "right";

    std::string disparityName = "disparity";
    /**
     * Input left image
     */
    Input& left = inputs[leftInputName];

    /**
     * Input right image
     */
    Input& right = inputs[rightInputName];

    /**
     *  Disparity
     */
    Input& disparity = inputs[disparityName];
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
