#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/utility/AtomicBool.hpp"
#include "depthai/utility/JoiningThread.hpp"
#include <spdlog/async_logger.h>

namespace dai {

class ThreadedNode : public Node {
   private:
    JoiningThread thread;
    AtomicBool running{false};

   public:
    using Node::Node;
    virtual ~ThreadedNode() = default;

    // override the following methods
    virtual void start() override;
    virtual void wait() override;
    virtual void stop() override;

    // virtual 'run' method
    virtual void run() = 0;

    // check if still running
    bool isRunning() const;

    std::shared_ptr<spdlog::async_logger> logger;
};

}  // namespace dai
