#pragma once

#include "depthai/log/LogLevel.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/utility/AtomicBool.hpp"
#include "depthai/utility/JoiningThread.hpp"
#include "depthai/utility/spimpl.h"

namespace dai {

class ThreadedNode : public Node {
    friend class PipelineImpl;
    friend class utility::PipelineImplHelper;

   private:
    JoiningThread thread;
    AtomicBool running{false};

   protected:
    void initPipelineEventDispatcher(int64_t nodeId);

   public:
    Output pipelineEventOutput{*this, {"pipelineEventOutput", DEFAULT_GROUP, {{{DatatypeEnum::PipelineEvent, false}}}}};

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

    bool mainLoop();

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

    utility::PipelineEventDispatcherInterface::BlockPipelineEvent inputBlockEvent();
    utility::PipelineEventDispatcherInterface::BlockPipelineEvent outputBlockEvent();
    utility::PipelineEventDispatcherInterface::BlockPipelineEvent blockEvent(PipelineEvent::Type type, const std::string& source);

    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
