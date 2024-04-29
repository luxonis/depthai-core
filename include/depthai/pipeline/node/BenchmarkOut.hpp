#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/BenchmarkPropertiesOut.hpp>

namespace dai {
namespace node {

class BenchmarkOut : public DeviceNodeCRTP<DeviceNode, BenchmarkOut, BenchmarkPropertiesOut> {
   public:
    constexpr static const char* NAME = "BenchmarkOut";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    std::shared_ptr<BenchmarkOut> build() {
        properties.numMessages = -1;  // By default send messages indefinitely
        isBuild = true; 
        return std::static_pointer_cast<BenchmarkOut>(shared_from_this());
    }

    /**
     * Send messages out as fast as possible
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::Buffer, true}}}};

    /**
     * Message that will be sent repeatedly
     */
    Input input{*this, {.name = "input", .blocking = true, .queueSize = 1, .types = {{DatatypeEnum::Buffer, true}}}};

    /**
     * Sets number of messages to send, by default send messages indefinitely
     * @param num number of messages to send
     */
    void setNumMessagesToSend(int num);

    /**
     * Set FPS at which the node is sending out messages. 0 means as fast as possible
     */
    void setFps(float fps);
};

}  // namespace node
}  // namespace dai
