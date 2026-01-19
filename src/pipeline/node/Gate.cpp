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
    while(true) {
        auto control = inputControl.tryGet<GateControl>();
        if(control) {
            return control;
        }
        auto inData = input.get();
        output.send(inData);
    }
}

std::shared_ptr<GateControl> Gate::sendMessages(int numMessages) {
    for(int i = 0; i < numMessages; i++) {
        auto inData = input.get();
        output.send(inData);
    }
    return std::make_shared<GateControl>(false, -1);
}

std::shared_ptr<GateControl> Gate::waitFotCommand() {
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
            currentCommand = waitFotCommand();
        }
    }
}

}  // namespace node
}  // namespace dai
