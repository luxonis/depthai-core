#pragma once

// std

// shared
#include "depthai/pipeline/PipelineSchema.hpp"

// libraries
#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "CallbackSink.hpp"

// shared
#include "depthai/log/LogConstants.hpp"

// project
#include "pipeline/GstMonoCamera.hpp"
// #include "pipeline/PipelineRequirements.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {
namespace gate {

class PipelineBuilder {
   public:
    using NodeBuildCb = std::function<std::shared_ptr<ThreadedNode>(Pipeline&, const NodeObjInfo&)>;
    using NodeHandler = std::unordered_map<std::string, NodeBuildCb>;

    static PipelineBuilder& getInstance();

   private:
    NodeHandler nodeHandler;
    std::vector<std::shared_ptr<GstMonoCamera>> monoCameraHandler;
    // std::vector<std::shared_ptr<GstColorCamera>> colorCameraHandler;

    volatile spdlog::level::level_enum currentLogLevel = spdlog::level::debug;
    volatile spdlog::level::level_enum currentLogLevelLoggers = spdlog::level::warn;
    std::function<void(dai::LogMessage)> logEventCallback;
    std::vector<spdlog::sink_ptr> sinks;
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> stdoutSink;
    std::shared_ptr<CallbackSink_mt> callbackSink;
    std::unordered_map<int64_t, std::shared_ptr<spdlog::logger>> nodeLoggers;
    std::shared_ptr<spdlog::logger> systemLogger;

    // Per-node logging level, if not set, use global logging level
    std::unordered_map<int64_t, spdlog::level::level_enum> nodeIdToLogLevel;

    PipelineBuilder();
    ~PipelineBuilder();

   public:
    static NodeHandler getDefaultNodeHandler();

    void setNodeHandler(NodeHandler handler);
    void init();
    void build(const PipelineSchema& schema, Pipeline& pipeline);
    // PipelineRequirements getRequirements(const Pipeline& pipeline);
    void start(Pipeline& pipeline);

    std::shared_ptr<spdlog::logger> getSystemLogger();

    void setLogEventCallback(std::function<void(dai::LogMessage)> cb);

    // set logging level of all node logger
    void setLogLevel(dai::LogLevel level);
    // set logging level of callback for all node loggers
    void setLogLevelCallback(dai::LogLevel level);
    // set logging level of a specific node logger
    void setNodeLogLevelCallback(int64_t nodeId, dai::LogLevel level);

    // get logging level of all node loggers
    dai::LogLevel getLogLevel();
    // get logging level of callback for all node loggers
    dai::LogLevel getLogLevelCallback();
    // get logging level of a specific node logger
    dai::LogLevel getNodeLogLevelCallback(int64_t nodeId);
};

}  // namespace gate
}  // namespace dai
