#pragma once
// project
#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>

namespace dai {
namespace node {
namespace test {

/**
 * @brief XLinkOut node. Sends messages over XLink.
 */
class MyConsumer : public NodeCRTP<ThreadedHostNode, MyConsumer> {
   public:
    constexpr static const char* NAME = "MyConsumer";

    /**
     * Input for any type of messages to be transferred over XLink stream
     * Default queue is blocking with size 8
     */
    Input input{*this, {"in", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, true}};

    void run() override {
        while(isRunning()) {
            auto msg = input.get<dai::Buffer>();
            std::cout << "got message (ptr: " << msg.get() << ", data (size: " << msg->data->getData().size() << "): ";

            for(int b : msg->getData()) {
                std::cout << b;
            }
            std::cout << "\n";
        }
    }
};

}  // namespace test
}  // namespace node
}  // namespace dai
