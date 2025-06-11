#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/BenchmarkOutProperties.hpp>

namespace dai {
namespace node {

class BenchmarkOut : public DeviceNodeCRTP<DeviceNode, BenchmarkOut, BenchmarkOutProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "BenchmarkOut";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Send messages out as fast as possible
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Message that will be sent repeatedly
     */
    Input input{*this, {"input", DEFAULT_GROUP, true, 1, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Sets number of messages to send, by default send messages indefinitely
     * @param num number of messages to send
     */
    void setNumMessagesToSend(int num);

    /**
     * Set FPS at which the node is sending out messages. 0 means as fast as possible
     */
    void setFps(float fps);

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
