#pragma once

#include <spdlog/async_logger.h>

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {

class HostNode : public ThreadedNode {
   public:
    constexpr static const char* NAME = "HostNode";
    // TODO(Morato) for now, this is a thin layer above threaded node.
    using ThreadedNode::ThreadedNode;
};

}  // namespace dai
