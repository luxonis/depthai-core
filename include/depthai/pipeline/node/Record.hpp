#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai/properties/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>

namespace dai {
namespace node {
namespace test {

/**
 * @brief XLinkOut node. Sends messages over XLink.
 */
class Record : public NodeCRTP<ThreadedNode, Record> {
   public:
    constexpr static const char* NAME = "Record";
    void build();

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input input{true, *this, "in", Input::Type::SReceiver, true, 8, true, {{DatatypeEnum::Buffer, true}}};

    void run() override {
        while(isRunning()) {
            auto msg = input.queue.get<dai::Buffer>();
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
