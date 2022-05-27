#include "depthai/pipeline/DeviceNode.hpp"


namespace dai
{

void DeviceNode::run() {

    auto pipeline = parent.lock();
    if(pipeline != nullptr) {
        // tmp tmp, get device, get corresponding IO -> string stream name
    }

}

} // namespace dai
