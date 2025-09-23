#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "PipelineEventDispatcherInterface.hpp"

namespace dai {
namespace utility {

class PipelineEventDispatcher : public PipelineEventDispatcherInterface {
    struct EventStatus {
        PipelineEvent::EventType type;
        std::chrono::microseconds duration;
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
        bool ongoing;
        std::optional<Buffer> metadata;
    };

    int64_t nodeId = -1;
    std::unordered_map<std::string, EventStatus> events;
    Node::Output* out = nullptr;

    void checkNodeId();

    uint32_t sequenceNum = 0;

   public:
    PipelineEventDispatcher() = delete;
    PipelineEventDispatcher(Node::Output* output) : out(output) {}

    void setNodeId(int64_t id) override;

    void addEvent(const std::string& source, PipelineEvent::EventType type) override;

    void startEvent(const std::string& source, std::optional<Buffer> metadata = std::nullopt) override;  // Start event with a start and an end
    void endEvent(const std::string& source) override;                                                   // Stop event with a start and an end
    void pingEvent(const std::string& source) override;                                                  // Event where stop and start are the same (eg. loop)
};

}  // namespace utility
}  // namespace dai
