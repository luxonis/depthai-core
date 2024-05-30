#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

namespace dai {

class InputQueue : public node::ThreadedHostNode {
    friend class Node::Input;

   public:
    /**
     * @brief Send a message to the connected input
     *
     * @param msg Message to send
     */
    void send(const std::shared_ptr<ADatatype>& msg);

   private:
    /**
     * @brief Construct a new Input Queue object
     *
     * @param maxSize: Maximum size of the input queue
     * @param blocking: Whether the input queue should block when full
     */
    InputQueue(unsigned int maxSize = 16, bool blocking = false);

    /**
     * @brief InputQueue's main thread function that takes care of sending messages from onhost to the connected ondevice input
     */
    void run() override;

    const char *getName() const override;

    Node::Input input{*this, {.name = "input", .types = {{DatatypeEnum::Buffer, true}}}};
    Node::Output output{*this, {.name = "output", .types = {{DatatypeEnum::Buffer, true}}}};

};

}  // namespace dai