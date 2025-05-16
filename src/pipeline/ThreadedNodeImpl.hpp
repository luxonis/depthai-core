#pragma once
#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {
class ThreadedNode::Impl {
   public:
    static inline std::shared_ptr<spdlog::details::thread_pool> threadPool = std::make_shared<spdlog::details::thread_pool>(8192, 1);
    std::shared_ptr<spdlog::async_logger> logger =
        std::make_shared<spdlog::async_logger>("ThreadedNode", std::make_shared<spdlog::sinks::stdout_color_sink_mt>(), threadPool);
};
}  // namespace dai
