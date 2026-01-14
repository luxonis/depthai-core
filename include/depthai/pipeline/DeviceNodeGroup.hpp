#pragma once

#include <memory>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"

namespace dai {

class DeviceNodeGroup : public DeviceNode {
   public:
    /**
     * Return the node name used by the pipeline.
     */
    const char* getName() const final {
        return "DeviceNodeGroup";
    }

    virtual ~DeviceNodeGroup();

    using DeviceNode::DeviceNode;
    /**
     * Construct a device node group attached to a device.
     */
    DeviceNodeGroup(const std::shared_ptr<Device>& device) : DeviceNode(device, std::make_unique<DeviceNodeGroupProperties>(), false) {}
    friend class PipelineImpl;

    /**
     * Set logging level for this node group.
     */
    void setLogLevel(dai::LogLevel level) override;
    /**
     * Get logging level for this node group.
     */
    dai::LogLevel getLogLevel() const override;
};

}  // namespace dai
