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
    Subnode(Node& parent, std::string alias) {
        if(!parent.configureMode) {
            node = std::make_shared<T>();
            node->setAlias(alias);

            if(auto deviceNode = std::dynamic_pointer_cast<DeviceNode>(node); deviceNode != nullptr) {
                if(dynamic_cast<DeviceNode*>(&parent) == nullptr) {
                    if(std::dynamic_pointer_cast<HostRunnable>(node) == nullptr) {
                        throw std::runtime_error("The parent node of a non HostRunnable DeviceNode must be a DeviceNode");
                    }
                } else {
                    deviceNode->setDevice(dynamic_cast<DeviceNode*>(&parent)->getDevice());
                }
            }

            parent.add(node);  // Node::add, not nodeMap.push_back: it runs adoptSubtree, else lazy subnodes get no parent

            node->buildInternal();
        }

        parent.nodeRefs.push_back(&node);
    }
    T& operator*() const noexcept {
        return *std::static_pointer_cast<T>(node).get();
    }
    T* operator->() const noexcept {
        return std::static_pointer_cast<T>(node).get();
    }
};

}  // namespace dai
