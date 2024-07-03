#pragma once

#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "depthai/pipeline/Node.hpp"
#include "depthai/utility/AtomicBool.hpp"
#include "depthai/utility/JoiningThread.hpp"

namespace dai {

class ThreadedNode : public Node {
   private:
    JoiningThread thread;
    AtomicBool running{false};
    static inline std::shared_ptr<spdlog::details::thread_pool> threadPool = std::make_shared<spdlog::details::thread_pool>(8192, 1);

    // Keep track of created inputs and outputs - we need to make sure they are kept
    // alive so that `getInputRefs` and `getOutputRefs` don't use dangling Input/Output
    // pointers
    std::vector<std::shared_ptr<Input>> createdInputs;
    std::vector<std::shared_ptr<Output>> createdOutputs;

   public:
    using Node::Node;
    ThreadedNode();
    virtual ~ThreadedNode() = default;

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

    /**
     * @brief Create an input for this node with default Input settings
     *
     * @return std::shared_ptr<Input>: A shared pointer to the created Input
     */
    std::shared_ptr<Input> createInput();

    /**
     * @brief Create an output for this node with default Output settings
     *
     * @return std::shared_ptr<Output>: A shared pointer to the created Output
     */
    std::shared_ptr<Output> createOutput();

    // check if still running
    bool isRunning() const;
    std::shared_ptr<spdlog::async_logger> logger =
        std::make_shared<spdlog::async_logger>("ThreadedNode", std::make_shared<spdlog::sinks::stdout_color_sink_mt>(), threadPool);
};

}  // namespace dai
