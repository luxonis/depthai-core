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

       public:
        BlockPipelineEvent(PipelineEventDispatcherInterface& dispatcher, PipelineEvent::Type type, const std::string& source)
            : dispatcher(dispatcher), type(type), source(source), sequence(dispatcher.sequence++) {
            dispatcher.startTrackedEvent(type, source, sequence);
        }
        ~BlockPipelineEvent() {
            PipelineEvent event;
            event.type = type;
            event.source = source;
            event.sequenceNum = sequence;
            event.queueSize = queueSize;
            if(!canceled) dispatcher.endTrackedEvent(type, source, sequence);
        }
        void cancel() {
            canceled = true;
        }
        void setQueueSize(uint32_t qs) {
            queueSize = qs;
        }
    };

    bool sendEvents = true;

    virtual ~PipelineEventDispatcherInterface() = default;
    virtual void setNodeId(int64_t id) = 0;
    virtual void startEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void startInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void startOutputEvent(const std::string& source) = 0;
    virtual void startCustomEvent(const std::string& source) = 0;
    virtual void endEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void endInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual void endOutputEvent(const std::string& source) = 0;
    virtual void endCustomEvent(const std::string& source) = 0;
    virtual void startTrackedEvent(PipelineEvent::Type type, const std::string& source, int64_t sequenceNum) = 0;
    virtual void startTrackedEvent(PipelineEvent event) = 0;
    virtual void endTrackedEvent(PipelineEvent::Type type, const std::string& source, int64_t sequenceNum) = 0;
    virtual void endTrackedEvent(PipelineEvent event) = 0;
    virtual void pingEvent(PipelineEvent::Type type, const std::string& source) = 0;
    virtual void pingMainLoopEvent() = 0;
    virtual void pingCustomEvent(const std::string& source) = 0;
    virtual void pingInputEvent(const std::string& source, int32_t status, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual BlockPipelineEvent blockEvent(PipelineEvent::Type type, const std::string& source) = 0;
    virtual BlockPipelineEvent inputBlockEvent() = 0;
    virtual BlockPipelineEvent outputBlockEvent() = 0;
    virtual BlockPipelineEvent customBlockEvent(const std::string& source) = 0;
};

}  // namespace utility
}  // namespace dai
