#pragma once

// project
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/Node.hpp"

namespace dai {

class NodeGroup : public Node {
   public:
    NodeGroup() = default;
    virtual ~NodeGroup() = default;

    const char* getName() const override {
        return "NodeGroup";
    };

    bool hasDeviceNodes() const {
        for(const auto& node : nodeMap) {
            if(dynamic_cast<DeviceNode*>(node.get())) {
                return true;
            }
        }
        return false;
    }

    void setDevice(std::shared_ptr<Device> device) {
        for(auto& node : nodeMap) {
            if(auto devNode = std::dynamic_pointer_cast<DeviceNode>(node)) {
                devNode->setDevice(device);
            }
        }
    }

    // std::unique_ptr<Node> clone() const override {
    //     return std::make_unique<NodeGroup>(static_cast<const NodeGroup&>(*this));
    // };
    friend class PipelineImpl;
};

}  // namespace dai
