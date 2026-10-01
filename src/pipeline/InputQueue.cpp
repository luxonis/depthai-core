#include "depthai/pipeline/InputQueue.hpp"

namespace dai {

void InputQueue::send(const std::shared_ptr<ADatatype>& msg) {
    inputQueueNode->send(msg);
}

bool InputQueue::trySend(const std::shared_ptr<ADatatype>& msg) {
    return inputQueueNode->trySend(msg);
}

void InputQueue::setPaused(bool paused) {
    inputQueueNode->input.setPaused(paused);
}

bool InputQueue::isPaused() const {
    return inputQueueNode->input.isPaused();
}

InputQueue::InputQueue(unsigned int maxSize, bool blocking) : inputQueueNode(std::make_shared<InputQueueNode>(maxSize, blocking)) {}

InputQueue::InputQueueNode::InputQueueNode(unsigned int maxSize, bool blocking) : ThreadedHostNode() {
    input.setBlocking(blocking);
    input.setMaxSize(maxSize);
}

void InputQueue::InputQueueNode::run() {
    while(mainLoop()) {
        output.send(input.get());
    }
}

void InputQueue::InputQueueNode::send(const std::shared_ptr<ADatatype>& msg) {
    input.send(msg);
}

bool InputQueue::InputQueueNode::trySend(const std::shared_ptr<ADatatype>& msg) {
    return input.trySend(msg);
}

const char* InputQueue::InputQueueNode::getName() const {
    return "InputQueue";
}

}  // namespace dai
