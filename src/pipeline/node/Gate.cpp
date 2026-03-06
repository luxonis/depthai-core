#include <chrono>
#include <depthai/pipeline/node/Gate.hpp>
#include <thread>

namespace dai {
namespace node {

Gate::Properties& Gate::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void Gate::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool Gate::runOnHost() const {
    return runOnHostVar;
}

Gate::Gate(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, Gate, GateProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

void Gate::sleepUntilNextSending(int fps, bool& started) {
    if(fps <= 0) return;

    auto interval = std::chrono::nanoseconds(1000000000LL / fps);
    auto now = std::chrono::steady_clock::now();

    if(!started) {
        nextTargetTime = now + interval;
        started = true;
        return;
    }
    if(now < nextTargetTime) {
        std::this_thread::sleep_until(nextTargetTime);
    }
    nextTargetTime += interval;
}

std::shared_ptr<GateControl> Gate::sendMessages(int fps) {
    bool started = false;
    while(mainLoop()) {
        if(MessageQueue::waitAny(inputs)) {
            if(auto inControl = inputControl.tryGet<GateControl>()) {
                return inControl;
            }
            if(auto in = input.tryGet()) {
                if(fps > 0) {
                    sleepUntilNextSending(fps, started);
                }
                output.send(in);
            }
        }
    }
    return nullptr;
}

std::shared_ptr<GateControl> Gate::sendMessages(int numMessages, int fps) {
    bool started = false;
    int sentMessages = 0;
    while(sentMessages < numMessages && mainLoop()) {
        if(MessageQueue::waitAny(inputs)) {
            if(!mainLoop()) break;

            if(auto inControl = inputControl.tryGet<GateControl>()) {
                return inControl;
            }
            if(auto in = input.tryGet()) {
                if(fps > 0) {
                    sleepUntilNextSending(fps, started);
                }
                output.send(in);
                sentMessages++;
            }
        }
    }

    // Auto-close logic after N messages
    auto closeCmd = std::make_shared<GateControl>();
    closeCmd->open = false;
    closeCmd->numMessages = -1;
    closeCmd->fps = -1;
    return closeCmd;
}

std::shared_ptr<GateControl> Gate::waitForCommand() {
    return inputControl.get<GateControl>();  // Blocking wait
}

void Gate::run() {
    auto currentCommand = std::make_shared<GateControl>(*initialConfig);

    // Initialize the timer baseline
    while(mainLoop()) {
        if(currentCommand->open) {
            if(currentCommand->numMessages >= 0) {
                // Pass both numMessages and fps
                currentCommand = sendMessages(currentCommand->numMessages, currentCommand->fps);
            } else {
                // Pass just fps for continuous mode
                currentCommand = sendMessages(currentCommand->fps);
            }
        } else {
            currentCommand = waitForCommand();
        }
    }
}

}  // namespace node
}  // namespace dai
