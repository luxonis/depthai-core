#pragma once

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {

class DeviceNode : public ThreadedNode {
   public:
    using ThreadedNode::ThreadedNode;
    virtual ~DeviceNode() = default;

    // virtual 'run' method
    virtual void run() override;
};

}  // namespace dai
