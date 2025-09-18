#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {
namespace utility {

class PipelineEventDispatcher {
    struct EventStatus {
        PipelineEvent::EventType type;
        std::chrono::microseconds duration;
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
        bool ongoing;
        std::optional<Buffer> metadata;
    };

    int64_t nodeId;
    std::unordered_map<std::string, EventStatus> events;
    Node::Output* out = nullptr;

   public:
    PipelineEventDispatcher(int64_t nodeId, Node::Output* output) : nodeId(nodeId), out(output) {}

    void addEvent(const std::string& source, PipelineEvent::EventType type);

    void startEvent(const std::string& source, std::optional<Buffer> metadata = std::nullopt);  // Start event with a start and an end
    void endEvent(const std::string& source);                                                   // Stop event with a start and an end
    void pingEvent(const std::string& source);                                                  // Event where stop and start are the same (eg. loop)
};

}  // namespace utility
}  // namespace dai
