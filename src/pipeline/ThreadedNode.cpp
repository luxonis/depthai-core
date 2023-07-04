#include "depthai/pipeline/ThreadedNode.hpp"

#include <spdlog/spdlog.h>

#include "utility/Platform.hpp"
namespace dai {

void ThreadedNode::start() {
    // Start the thread
    running = true;
    thread = std::thread([this]() {
        try {
            run();
        } catch(const MessageQueue::QueueException& ex) {
            // catch anything and stop the node
            logger->info("Node closing: {}", ex.what());
            running = false;
        } catch(const std::runtime_error& ex){
            logger->error("Node threw exception, stopping the node. Exception message: {}", ex.what());
            running = false;
        }
    });
    platform::setThreadName(thread, fmt::format("{}({})", getName(), id));
}

void ThreadedNode::wait() {
    if(thread.joinable()) thread.join();
}

void ThreadedNode::stop() {
    // TBD
    // Sets running to false
    running = false;
    // closes all the queueus, then waits for the thread to join
    for(auto& in : getInputRefs()) {
        in->queue.close();
    }
    // for(auto& rout : getOutputRefs()) {
    // }
    // wait();
}

bool ThreadedNode::isRunning() const {
    return running;
}

}  // namespace dai
