#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Node.hpp>
#include <memory>
#include <string>

namespace dai {

template <typename T>
class Subnode {
    std::shared_ptr<Node> node;

   public:
    /**
     * Construct a subnode attached to a parent node with an alias.
     */
    Subnode(Node& parent, std::string alias) {
        if(!parent.configureMode) {
            // Create node as well
            node = std::make_shared<T>();
            node->setAlias(alias);

            // Add node to parents map
            parent.nodeMap.push_back(node);

            // If node is DeviceNode, copy device from parent
            if(std::dynamic_pointer_cast<DeviceNode>(node) != nullptr) {
                // Check if the parent is DeviceNode
                if(dynamic_cast<DeviceNode*>(&parent) == nullptr) {
                    if(std::dynamic_pointer_cast<HostRunnable>(node) == nullptr) {
                        throw std::runtime_error("The parent node of a non HostRunnable DeviceNode must be a DeviceNode");
                    }
                } else {
                    auto device = dynamic_cast<DeviceNode*>(&parent)->getDevice();
                    std::dynamic_pointer_cast<DeviceNode>(node)->setDevice(device);
                }
            }

            // Now that node is created, call buildInternal
            node->buildInternal();
        }

        // Add reference
        parent.nodeRefs.push_back(&node);
    }
    /**
     * Access the underlying node reference.
     */
    T& operator*() const noexcept {
        return *std::static_pointer_cast<T>(node).get();
    }
    /**
     * Access the underlying node pointer.
     */
    T* operator->() const noexcept {
        return std::static_pointer_cast<T>(node).get();
    }
};

}  // namespace dai
