#include "depthai/pipeline/DeviceNodeGroup.hpp"

namespace dai {

void DeviceNodeGroup::setDeviceForAllDeviceNodes(const std::shared_ptr<Device>& device) {
    for(auto& node : DeviceNode::getNodeMap()) {
        if(std::dynamic_pointer_cast<DeviceNode>(node) != nullptr) {
            std::dynamic_pointer_cast<DeviceNode>(node)->setDevice(device);
        }
    }
}

void DeviceNodeGroup::buildInternal() {
    for(auto& node : DeviceNode::getNodeMap()) {
        node->buildInternal();
    }
}

}  // namespace dai