#include "depthai/pipeline/InputQueue.hpp"

namespace dai {

void InputQueue::send(const std::shared_ptr<ADatatype>& msg) {
    inputQueueNode->send(msg);
}

InputQueue::InputQueue(unsigned int maxSize, bool blocking) : inputQueueNode(std::make_shared<InputQueueNode>(maxSize, blocking)) {}

InputQueue::InputQueueNode::InputQueueNode(unsigned int maxSize, bool blocking) : ThreadedHostNode() {
    input.setBlocking(blocking);
    input.setMaxSize(maxSize);
}

void InputQueue::InputQueueNode::run() {
    while(isRunning()) {
        output.send(input.get());
    }
}

void InputQueue::InputQueueNode::send(const std::shared_ptr<ADatatype>& msg) {
    input.send(msg);
}

const char* InputQueue::InputQueueNode::getName() const {
    return "InputQueue";
}

}  // namespace dai