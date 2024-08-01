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

    void buildInternal() override;

   protected:
    // Set device for all device nodes in this node group
    void setDeviceForAllDeviceNodes(const std::shared_ptr<Device>& device);
};

}  // namespace dai
