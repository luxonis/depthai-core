#pragma once

#include <memory>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"

namespace dai {

template <typename T>
class DeviceNodeGroup : public DeviceNodeCRTP<DeviceNode, T, DeviceNodeGroupProperties> {
   public:
    constexpr static const char* NAME = "DeviceNodeGroup";
    using DeviceNodeCRTP<DeviceNode, T, DeviceNodeGroupProperties>::DeviceNodeCRTP;
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
