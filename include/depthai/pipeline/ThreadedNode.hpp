#pragma once

#include "depthai/pipeline/Node.hpp"

#include "depthai/utility/JoiningThread.hpp"

namespace dai {

class ThreadedNode : public Node
{

private:
    JoiningThread thread;
public:
    using Node::Node;

    // override the following methods
    virtual void start() override;
    virtual void wait() override;
    virtual void stop() override;

    // virtual 'run' method
    virtual void run();
};

};


} // namespace dai
