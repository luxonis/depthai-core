#pragma once

#include "depthai/log/LogLevel.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/utility/AtomicBool.hpp"
#include "depthai/utility/JoiningThread.hpp"
#include "depthai/utility/spimpl.h"

namespace dai {

class ThreadedNode : public Node {
   private:
    JoiningThread thread;
    AtomicBool running{false};

   public:
    using Node::Node;
    ThreadedNode();
    virtual ~ThreadedNode();

    /**
     * @brief Function called at the beginning of the `start` function.
     *
     * This function may be overridden by the user to perform any needed tasks prior
     * to starting this node's main thread.
     */
    virtual void onStart() {}

    /**
     * @brief Function called at the end of the `stop` function.
     *
     * This function may be overridden by the user to perform any needed tasks
     * directly after stopping this node's main thread.
     */
    virtual void onStop() {}

    // override the following methods
    void start() override;
    void wait() override;
    void stop() override;

    // virtual 'run' method
    virtual void run() = 0;

    // check if still running
    bool isRunning() const;

    /**
     * @brief Sets the logging severity level for this node.
     *
     * @param level Logging severity level
     */
    virtual void setLogLevel(dai::LogLevel level);

    /**
     * @brief Gets the logging severity level for this node.
     *
     * @returns Logging severity level
     */
    virtual dai::LogLevel getLogLevel() const;

    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
