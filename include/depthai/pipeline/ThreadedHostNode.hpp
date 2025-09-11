#pragma once

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {
namespace node {
class ThreadedHostNode : public ThreadedNode {
   public:
    constexpr static const char* NAME = "HostNode";
    using ThreadedNode::ThreadedNode;

    ~ThreadedHostNode() override;

    bool runOnHost() const final {
        // Host node don't contain the necessary information to be serialized and sent to the device
        return true;
    }
};

/**
 * @brief Custom node for host node. When creating a custom host node, inherit from this class!
 * @tparam T Node type (same as the class you are creating)
 *
 * Example:
 * @code{.cpp}
 * class MyNode : public CustomThreadedNode<MyNode> {
 *    public:
 *        void run() override {
 *            // ...
 *        }
 * };
 * @endcode
 */
template <typename T>
using CustomThreadedNode = NodeCRTP<ThreadedHostNode, T>;
}  // namespace node
}  // namespace dai
