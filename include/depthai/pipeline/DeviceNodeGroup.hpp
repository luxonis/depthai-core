#pragma once

#include <memory>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"

namespace dai {

class DeviceNodeGroup : public DeviceNode {
   public:
    using DeviceNode::DeviceNode;
    DeviceNodeGroup(const std::shared_ptr<Device>& device) : DeviceNode(device, std::make_unique<DeviceNodeGroupProperties>(), false) {}
    friend class PipelineImpl;

   protected:
    // Set device for all device nodes in this node group
    void setDeviceForAllDeviceNodes(const std::shared_ptr<Device>& device) {
        for(auto& node : DeviceNode::getNodeMap()) {
            if(std::dynamic_pointer_cast<DeviceNode>(node) != nullptr) {
                std::dynamic_pointer_cast<DeviceNode>(node)->setDevice(device);
            }
        }
    }
};

}  // namespace dai
