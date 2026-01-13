#include <depthai/pipeline/node/Gate.hpp>

namespace dai {
namespace node {

void Gate::run() {
    bool startStopSwitch = true;  // true start, false stop
    while(mainLoop()) {
        auto control = inputControl.tryGet<GateControl>();
        if(control) {
            startStopSwitch = control->value;
        }

        if(startStopSwitch) {
            auto inData = input.get();
            output.send(inData);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(properties.sleepingTimeMs));
        }
    }
}

}  // namespace node
}  // namespace dai
