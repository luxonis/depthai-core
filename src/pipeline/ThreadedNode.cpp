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
            // catch the exception and stop the node
            auto expStr = fmt::format("Node stopped with a queue exception: {}", ex.what());
            if(logger) {
                logger->info(expStr);
            } else {
                spdlog::info(expStr);
            }
            running = false;
        } catch(const std::runtime_error& ex) {
            auto expStr = fmt::format("Node threw exception, stopping the node. Exception message: {}", ex.what());
            if(logger) {
                logger->error(expStr);
            } else {
                spdlog::error(expStr);
            }
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
