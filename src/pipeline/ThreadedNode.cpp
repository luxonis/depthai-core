#include "depthai/pipeline/ThreadedNode.hpp"


namespace dai
{

void ThreadedNode::start() {
    // Start the thread
    thread = std::thread([this](){
        run();
    });
}

void ThreadedNode::wait() {
    thread = std::thread{};
}

void ThreadedNode::stop() {
    // TBD
    // closes all the queueus, then waits for the thread to join
}

} // namespace dai
