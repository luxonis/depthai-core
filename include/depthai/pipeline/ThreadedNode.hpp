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

   public:
    using Node::Node;
    ThreadedNode();
    virtual ~ThreadedNode() = default;

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
};

}  // namespace dai
