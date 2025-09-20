#pragma once

#include <memory>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"
#include "depthai/utility/export.hpp"

namespace dai {

class DEPTHAI_API DeviceNodeGroup : public DeviceNode {
   public:
    const char* getName() const final {
        return "DeviceNodeGroup";
    }

    using DeviceNode::DeviceNode;
    DeviceNodeGroup(const std::shared_ptr<Device>& device) : DeviceNode(device, std::make_unique<DeviceNodeGroupProperties>(), false) {}
    friend class PipelineImpl;

    void setLogLevel(dai::LogLevel level) override;
    dai::LogLevel getLogLevel() const override;
};

}  // namespace dai
