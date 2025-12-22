#pragma once

#include <atomic>
#include <optional>

#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {
namespace utility {

class PipelineEventDispatcherInterface {
    std::atomic<uint64_t> sequence{0};

   public:
    class BlockPipelineEvent {
        PipelineEventDispatcherInterface& dispatcher;
        PipelineEvent::Type type;
        std::string source;
        uint64_t sequence;

        bool canceled = false;
        std::optional<uint32_t> queueSize = std::nullopt;
        std::optional<std::chrono::time_point<std::chrono::steady_clock>> endTs = std::nullopt;

       public:
        BlockPipelineEvent(PipelineEventDispatcherInterface& dispatcher,
                           PipelineEvent::Type type,
                           const std::string& source,
                           std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt)
            : dispatcher(dispatcher), type(type), source(source), sequence(dispatcher.sequence++) {
            PipelineEvent event;
            event.type = type;
            event.source = source;
            event.sequenceNum = sequence;
            dispatcher.startTrackedEvent(event, ts);
        }
        void destroy() {
            if(canceled || std::uncaught_exceptions() > 0) return;
            PipelineEvent event;
            event.type = type;
            event.source = source;
            event.sequenceNum = sequence;
            event.queueSize = queueSize;
            dispatcher.endTrackedEvent(event, endTs);
        }
        void cancel() {
            canceled = true;
        }
        void setQueueSize(uint32_t qs) {
            queueSize = qs;
        }
        void setEndTimestamp(std::chrono::time_point<std::chrono::steady_clock> ts) {
            endTs = ts;
        }
        ~BlockPipelineEvent() {
            destroy();
        }
    };

    bool sendEvents = true;

    virtual ~PipelineEventDispatcherInterface();
    virtual void startEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void startInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void startOutputEvent(const std::string& source) = 0;
    virtual void startCustomEvent(const std::string& source) = 0;
    virtual void endEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void endInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void endOutputEvent(const std::string& source) = 0;
    virtual void endCustomEvent(const std::string& source) = 0;
    virtual void startTrackedEvent(PipelineEvent event, std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) = 0;
    virtual void startTrackedEvent(PipelineEvent::Type type,
                                   const std::string& source,
                                   int64_t sequenceNum,
                                   std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) = 0;
    virtual void endTrackedEvent(PipelineEvent event, std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) = 0;
    virtual void endTrackedEvent(PipelineEvent::Type type,
                                 const std::string& source,
                                 int64_t sequenceNum,
                                 std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) = 0;
    virtual void pingEvent(PipelineEvent::Type type, const std::string& source) = 0;
    virtual void pingMainLoopEvent() = 0;
    virtual void pingCustomEvent(const std::string& source) = 0;
    virtual void pingInputEvent(const std::string& source, PipelineEvent::Status status, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual BlockPipelineEvent blockEvent(PipelineEvent::Type type,
                                          const std::string& source,
                                          std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) = 0;
    virtual BlockPipelineEvent inputBlockEvent() = 0;
    virtual BlockPipelineEvent outputBlockEvent() = 0;
    virtual BlockPipelineEvent customBlockEvent(const std::string& source) = 0;
};

}  // namespace utility
}  // namespace dai
