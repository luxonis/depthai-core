#include <depthai/pipeline/node/Gate.hpp>

namespace dai {
namespace node {

void Gate::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool Gate::runOnHost() const {
    return runOnHostVar;
}

void Gate::run() {
    bool startStopSwitch = true;  // true start, false stop
    while(mainLoop()) {
        if(startStopSwitch) {
            auto control = inputControl.tryGet<GateControl>();
            if(control) {
                startStopSwitch = control->value;
            }

            auto inData = input.get();
            output.send(inData);
        } else {
            auto control = inputControl.get<GateControl>();
            if(control) {
                startStopSwitch = control->value;
            }
        }
    }
}

}  // namespace node
}  // namespace dai
