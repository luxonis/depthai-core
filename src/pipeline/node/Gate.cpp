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


void Gate::updateLastSentTime() {
    lastSentTime = std::chrono::steady_clock::now();
}

void Gate::sleepUntilNextSending(int fps) {
    if(fps <= 0) return;

    auto currentTime = std::chrono::steady_clock::now();
    auto interval = std::chrono::nanoseconds(1000000000LL / fps);
    auto elapsed = currentTime - lastSentTime;

    if(elapsed < interval) {
        std::this_thread::sleep_for(interval - elapsed);
    }
}


std::shared_ptr<GateControl> Gate::sendMessages(int fps) {
    while(mainLoop()) {
        if(MessageQueue::waitAny(inputs)) {
            if(auto inControl = inputControl.tryGet<GateControl>()) {
                return inControl;
            }
            if(auto in = input.tryGet()) {
                if(fps > 0) {
                    sleepUntilNextSending(fps);
                }
                output.send(in);
                updateLastSentTime();
            }
        }
    }
    return nullptr;
}

std::shared_ptr<GateControl> Gate::sendMessages(int numMessages, int fps) {
    int sentMessages = 0;
    while(sentMessages < numMessages && mainLoop()) {
        if(MessageQueue::waitAny(inputs)) {
            if(!mainLoop()) break;

            if(auto inControl = inputControl.tryGet<GateControl>()) {
                return inControl;
            }
            if(auto in = input.tryGet()) {
                if(fps > 0) {
                    sleepUntilNextSending(fps);
                }
                output.send(in);
                updateLastSentTime();
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
    updateLastSentTime();

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
