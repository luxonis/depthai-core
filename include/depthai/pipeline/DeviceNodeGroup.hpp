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

    using DeviceNode::DeviceNode;
    DeviceNodeGroup(const std::shared_ptr<Device>& device) : DeviceNode(device, std::make_unique<DeviceNodeGroupProperties>(), false) {}
    friend class PipelineImpl;

    void setLogLevel(dai::LogLevel level) override {
        for(auto& node : nodeRefs) {
            auto threadedNode = std::dynamic_pointer_cast<ThreadedNode>(*node);
            if(threadedNode) {
                threadedNode->setLogLevel(level);
            }
        }
    }

    dai::LogLevel getLogLevel() const override {
        for(auto& node : nodeRefs) {
            auto threadedNode = std::dynamic_pointer_cast<ThreadedNode>(*node);
            if(threadedNode) {
                return threadedNode->getLogLevel();
            }
        }

        // Should not be able to get here
        return dai::LogLevel::OFF;
    }
};

}  // namespace dai
