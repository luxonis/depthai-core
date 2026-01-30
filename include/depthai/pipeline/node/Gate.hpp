#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/properties/GateProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief Gate Node.
 *
 * This node acts as a valve for data pipelines. It controls the flow of messages
 * from the 'input' to the 'output' based on the state configured via 'inputControl'.
 * It can be configured to stay open indefinitely, stay closed, or open for a
 * specific number of messages.
 */
class Gate : public DeviceNodeCRTP<DeviceNode, Gate, GateProperties> {
   protected:
    Properties& getProperties() override;

   public:
    Gate(std::unique_ptr<Properties> props);

    std::shared_ptr<GateControl> initialConfig = std::make_shared<GateControl>();

    constexpr static const char* NAME = "Gate";

    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * @brief Main data input.
     * * Accepts arbitrary Buffer messages (e.g., ImgFrame, NNData).
     * If the Gate is Open, messages received here are forwarded to 'output'.
     * If the Gate is Closed, messages received here are discarded/dropped.
     * * Default queue size: 4
     * Blocking: False
     */
    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * @brief Main data output.
     * * Forwards messages that were allowed through the Gate.
     * The data type matches the input message.
     */
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * @brief Control input.
     * * Accepts 'GateControl' messages to dynamically change the Gate's state.
     * Use this to Open/Close the gate or set it to pass a specific number of frames
     * at runtime.
     * * Default queue size: 4
     */
    Input inputControl{*this, {"inputControl", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::GateControl, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * @brief Check if the node is configured to run on the host.
     * @return true if running on host, false otherwise.
     */
    bool runOnHost() const override;

    void run() override;

   private:
    const std::vector<MessageQueue*> inputs = {&inputControl, &input};
    bool runOnHostVar = false;

    std::shared_ptr<GateControl> sendMessages();

    std::shared_ptr<GateControl> sendMessages(int numMessages);

    std::shared_ptr<GateControl> waitForCommand();
};

}  // namespace node
}  // namespace dai
