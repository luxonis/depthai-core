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

    std::shared_ptr<BenchmarkOut> build();

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

   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai
