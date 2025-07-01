#include "depthai/pipeline/InputQueue.hpp"

#include <utility>

namespace dai {

void InputQueue::send(const std::shared_ptr<ADatatype>& msg) {
    inputQueueNode->send(msg);
}

InputQueue::InputQueue(std::string name, unsigned int maxSize, bool blocking) : inputQueueNode(std::make_shared<InputQueueNode>(name, maxSize, blocking)) {}

InputQueue::InputQueueNode::InputQueueNode(std::string name, unsigned int maxSize, bool blocking) : ThreadedHostNode() {
    input.setName(std::move(name));
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
    return name.c_str();
}

}  // namespace dai
