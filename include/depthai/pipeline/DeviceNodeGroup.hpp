#pragma once

#include <memory>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"

namespace dai {

class DeviceNodeGroup : public DeviceNode {
   public:
    const char* getName() const final {
        return "DeviceNodeGroup";
    }

    virtual ~DeviceNodeGroup();

    using DeviceNode::DeviceNode;
    DeviceNodeGroup(const std::shared_ptr<Device>& device) : DeviceNode(device, std::make_unique<DeviceNodeGroupProperties>(), false) {}
    friend class PipelineImpl;

    void setLogLevel(dai::LogLevel level) override;
    dai::LogLevel getLogLevel() const override;
};

}  // namespace dai
