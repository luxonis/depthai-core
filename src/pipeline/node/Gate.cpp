#include <depthai/pipeline/node/Gate.hpp>

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

std::shared_ptr<GateControl> Gate::sendMessages() {
    while(mainLoop()) {
        if(MessageQueue::waitAny(inputs)) {
            if(auto inControl = inputControl.tryGet<GateControl>()) {
                return inControl;
            }
            if(auto in = input.tryGet()) {
                output.send(in);
            }
        }
    }
    return nullptr;
}

std::shared_ptr<GateControl> Gate::sendMessages(int numMessages) {
    for(int i = 0; i < numMessages; i++) {
        if(MessageQueue::waitAny(inputs)) {
            if(!mainLoop()) break;

            if(auto inControl = inputControl.tryGet<GateControl>()) {
                return inControl;
            }
            if(auto in = input.tryGet()) {
                output.send(in);
            }
        }
    }
    return std::make_shared<GateControl>(false, -1);
}

std::shared_ptr<GateControl> Gate::waitForCommand() {
    auto control = inputControl.get<GateControl>();
    return control;
}

void Gate::run() {
    auto currentCommand = std::make_shared<GateControl>(*initialConfig);

    while(mainLoop()) {
        if(currentCommand->open) {
            if(currentCommand->numMessages >= 0) {
                currentCommand = sendMessages(currentCommand->numMessages);
            } else {
                currentCommand = sendMessages();
            }
        } else {
            currentCommand = waitForCommand();
        }
    }
}

}  // namespace node
}  // namespace dai
