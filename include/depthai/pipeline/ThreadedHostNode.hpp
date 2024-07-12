#pragma once

#include <spdlog/async_logger.h>

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {
namespace node {
class ThreadedHostNode : public ThreadedNode {
   public:
    constexpr static const char* NAME = "HostNode";
    using ThreadedNode::ThreadedNode;
    bool runOnHost() const final {
        // Host node don't contain the necessary information to be serialized and sent to the device
        return true;
    }
};

template <typename T>
using CustomThreadedNode = NodeCRTP<ThreadedHostNode, T>;
}  // namespace node
}  // namespace dai
