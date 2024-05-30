#include "depthai/pipeline/InputQueue.hpp"

namespace dai {

InputQueue::InputQueue(unsigned int maxSize, bool blocking) : ThreadedHostNode() {
    input.setBlocking(blocking);
    input.setMaxSize(maxSize);
}

void InputQueue::run() {
    while(isRunning()) {
        output.send(input.get());
    }
}

void InputQueue::send(const std::shared_ptr<ADatatype>& msg) {
    input.send(msg);
}

const char* InputQueue::getName() const {
    return "InputQueue";
}

}  // namespace dai