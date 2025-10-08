#pragma once

#include <optional>

#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {
namespace utility {

class PipelineEventDispatcherInterface {
   public:
    class BlockPipelineEvent {
        PipelineEventDispatcherInterface& dispatcher;
        PipelineEvent::Type type;
        std::string source;

       public:
        BlockPipelineEvent(PipelineEventDispatcherInterface& dispatcher, PipelineEvent::Type type, const std::string& source)
            : dispatcher(dispatcher), type(type), source(source) {
            dispatcher.startEvent(type, source, std::nullopt);
        }
        ~BlockPipelineEvent() {
            dispatcher.endEvent(type, source, std::nullopt);
        }
    };

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
    virtual void pingEvent(PipelineEvent::Type type, const std::string& source) = 0;
    virtual void pingMainLoopEvent() = 0;
    virtual void pingCustomEvent(const std::string& source) = 0;
    virtual void pingInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) = 0;
    virtual BlockPipelineEvent blockEvent(PipelineEvent::Type type, const std::string& source) = 0;
    virtual BlockPipelineEvent inputBlockEvent(const std::string& source = "defaultInputGroup") = 0;
    virtual BlockPipelineEvent outputBlockEvent(const std::string& source = "defaultOutputGroup") = 0;
    virtual BlockPipelineEvent customBlockEvent(const std::string& source) = 0;
};

}  // namespace utility
}  // namespace dai
