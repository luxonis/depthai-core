#include "depthai/pipeline/DeviceNodeGroup.hpp"

namespace dai {

DeviceNodeGroup::~DeviceNodeGroup() = default;

void DeviceNodeGroup::setLogLevel(dai::LogLevel level) {
    for(auto& node : nodeRefs) {
        // Try converting to ThreadedNode
        auto threadedNode = std::dynamic_pointer_cast<ThreadedNode>(*node);

        // Subnodes may not necessarily be ThreadedNodes
        if(threadedNode) {
            threadedNode->setLogLevel(level);
        }
    }
}

dai::LogLevel DeviceNodeGroup::getLogLevel() const {
    throw std::runtime_error("Cannot get log level for DeviceNodeGroup. Use getLogLevel() of a specific subnode instead.");
}

}  // namespace dai