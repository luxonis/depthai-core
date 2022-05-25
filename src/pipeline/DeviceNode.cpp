#include "depthai/pipeline/DeviceNode.hpp"


namespace dai
{

void DeviceNode::start() {
    // Start the thread
    thread = std::thread([this](){
        run();
    });
}

void DeviceNode::wait() {
    thread = std::thread{};
}

void DeviceNode::stop() {
    // TBD
    // closes all the queueus, then waits for the thread to join
}

void DeviceNode::run() {


}

} // namespace dai
