#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "PipelineEventDispatcherInterface.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {
namespace utility {

class PipelineEventDispatcher : public PipelineEventDispatcherInterface {
    struct EventStatus {
        PipelineEvent event;
        bool ongoing;
    };

    int64_t& nodeId;
    std::unordered_map<std::string, EventStatus> events;
    Node::Output* out = nullptr;

    void checkNodeId();

    uint32_t sequenceNum = 0;

    std::mutex mutex;

   public:
    PipelineEventDispatcher() = delete;
    PipelineEventDispatcher(int64_t& nodeId, Node::Output* output) : nodeId(nodeId), out(output) {}

    void startEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) override;
    void startInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) override;
    void startOutputEvent(const std::string& source) override;
    void startCustomEvent(const std::string& source) override;
    void endEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) override;
    void endInputEvent(const std::string& source, std::optional<uint32_t> queueSize = std::nullopt) override;
    void endOutputEvent(const std::string& source) override;
    void endCustomEvent(const std::string& source) override;
    // Timestamp in event is not used. If ts is specified ts is used, else curent time is applied
    void startTrackedEvent(PipelineEvent event, std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) override;
    void startTrackedEvent(PipelineEvent::Type type,
                           const std::string& source,
                           int64_t sequenceNum,
                           std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) override;
    // Timestamp in event is not used. If ts is specified ts is used, else curent time is applied
    void endTrackedEvent(PipelineEvent event, std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) override;
    void endTrackedEvent(PipelineEvent::Type type,
                         const std::string& source,
                         int64_t sequenceNum,
                         std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) override;
    void pingEvent(PipelineEvent::Type type, const std::string& source) override;
    void pingMainLoopEvent() override;
    void pingCustomEvent(const std::string& source) override;
    void pingInputEvent(const std::string& source, PipelineEvent::Status status, std::optional<uint32_t> queueSize = std::nullopt) override;
    BlockPipelineEvent blockEvent(PipelineEvent::Type type,
                                  const std::string& source,
                                  std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts = std::nullopt) override;
    BlockPipelineEvent inputBlockEvent() override;
    BlockPipelineEvent outputBlockEvent() override;
    BlockPipelineEvent customBlockEvent(const std::string& source) override;
};

}  // namespace utility
}  // namespace dai
