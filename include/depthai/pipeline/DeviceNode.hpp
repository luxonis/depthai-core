#pragma once

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {

class DeviceNode : public ThreadedNode
{

public:
    using ThreadedNode::ThreadedNode;

    // override the following methods
    virtual void start() override;
    virtual void wait() override;
    virtual void stop() override;

    // virtual 'run' method
    virtual void run() override;
};

};


} // namespace dai
