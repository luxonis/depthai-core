#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Sync.hpp"

// shared
#include <depthai/properties/RectificationProperties.hpp>

namespace dai {
namespace node {

class Rectification : public DeviceNodeCRTP<DeviceNode, Rectification, RectificationProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "Rectification";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;
    std::string input1Name = "input1";
    std::string input2Name = "input2";
    /**
     * Input left image
     */
    Input& input1 = inputs[input1Name];

    /**
     * Input right image
     */
    Input& input2 = inputs[input2Name];

    void buildInternal() override;

    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 4, {{DatatypeEnum::MessageGroup, true}}}};

    /**
     * Passthrough for input messages (so the node can be placed between other nodes)
     */
    Output passthrough1{*this, {"passthrough1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output passthrough2{*this, {"passthrough2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Send outputs
     */
    Output output1{*this, {"output1", DEFAULT_GROUP, {{{DatatypeEnum::BenchmarkReport, false}}}}};
    Output output2{*this, {"output2", DEFAULT_GROUP, {{{DatatypeEnum::BenchmarkReport, false}}}}};

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

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai
