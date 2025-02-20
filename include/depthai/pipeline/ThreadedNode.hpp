#pragma once

#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "depthai/log/LogLevel.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/utility/AtomicBool.hpp"
#include "depthai/utility/JoiningThread.hpp"

namespace dai {

class ThreadedNode : public Node {
   private:
    JoiningThread thread;
    AtomicBool running{false};
    static inline std::shared_ptr<spdlog::details::thread_pool> threadPool = std::make_shared<spdlog::details::thread_pool>(8192, 1);

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

    // check if still running
    bool isRunning() const;
    std::shared_ptr<spdlog::async_logger> logger =
        std::make_shared<spdlog::async_logger>("ThreadedNode", std::make_shared<spdlog::sinks::stdout_color_sink_mt>(), threadPool);

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
};

}  // namespace dai
