#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

// shared
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/properties/VppProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief Vpp node. Apply Virtual Projection Pattern algorithm to stereo images based on disparity.
 */
class Vpp : public DeviceNodeCRTP<DeviceNode, Vpp, VppProperties> {
   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "Vpp";

    using DeviceNodeCRTP::DeviceNodeCRTP;

    Vpp();

    Vpp(std::unique_ptr<Properties> props);

    virtual ~Vpp();

    void buildInternal() override;

    /**
     * Initial config to use for VPP.
     */
    std::shared_ptr<VppConfig> initialConfig = std::make_shared<VppConfig>();

    Subnode<node::Sync> sync{*this, "sync"};
    Input syncedInputs{*this, {"syncedInputs", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::MessageGroup, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    const std::string leftInputName = "left";
    const std::string rightInputName = "right";
    const std::string disparityName = "disparity";
    const std::string confidenceName = "confidence";

    Input* left;
    Input* right;
    Input* disparity;
    Input* confidence;

    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::VppConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

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
