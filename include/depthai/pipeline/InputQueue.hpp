#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

namespace dai {

class InputQueue {
    friend class Node::Input;

   public:
    /**
     * @brief Send a message to the connected input
     *
     * @param msg: Message to send
     */
    void send(const std::shared_ptr<ADatatype>& msg);

   private:
    /**
     * @brief Construct a new Input Queue object. The constructor is private as we only want to expose the relevant methods - only send for now
     *
     * @param maxSize: Maximum size of the input queue
     * @param blocking: Whether the input queue should block when full
     */
    InputQueue(unsigned int maxSize = 16, bool blocking = false);

    class InputQueueNode : public node::ThreadedHostNode {
       public:
        /** Constructor*/
        InputQueueNode(unsigned int maxSize, bool blocking);

        /** Send message from host*/
        void send(const std::shared_ptr<ADatatype>& msg);

        void run() override;
        const char* getName() const override;

        Node::Input input{*this, {.name = "input", .types = {{DatatypeEnum::Buffer, true}}}};
        Node::Output output{*this, {.name = "output", .types = {{DatatypeEnum::Buffer, true}}}};
    };

    // Helper access functions
    inline Node::Output& getNodeOutput() {
        return inputQueueNode->output;
    }
    inline std::shared_ptr<Node> getNode() {
        return inputQueueNode;
    }

    /** Pointer to InputQueueNode that does the actual communication between host and device */
    std::shared_ptr<InputQueueNode> inputQueueNode;
};
}  // namespace dai