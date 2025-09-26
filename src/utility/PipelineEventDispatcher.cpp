#include "depthai/utility/PipelineEventDispatcher.hpp"
#include <optional>
#include <thread>

namespace dai {
namespace utility {

void PipelineEventDispatcher::checkNodeId() {
    if(nodeId == -1) {
        throw std::runtime_error("Node ID not set on PipelineEventDispatcher");
    }
}
void PipelineEventDispatcher::setNodeId(int64_t id) {
    nodeId = id;
}
void PipelineEventDispatcher::addEvent(const std::string& source, PipelineEvent::EventType type) {
    if(!source.empty()) {
        if(events.find(source) != events.end()) {
            throw std::runtime_error("Event with name '" + source + "' already exists");
        }
        events[source] = {type, {}, {}, false, std::nullopt};
    }
}
void PipelineEventDispatcher::startEvent(const std::string& source, std::optional<Buffer> metadata) {
    checkNodeId();
    if(events.find(source) == events.end()) {
        throw std::runtime_error("Event with name " + source + " does not exist");
    }
    auto& event = events[source];
    if(event.ongoing) {
        throw std::runtime_error("Event with name " + source + " is already ongoing");
    }
    event.timestamp = std::chrono::steady_clock::now();
    event.ongoing = true;
    event.metadata = metadata;
}
void PipelineEventDispatcher::endEvent(const std::string& source) {
    checkNodeId();
    auto now = std::chrono::steady_clock::now();

    if(events.find(source) == events.end()) {
        throw std::runtime_error("Event with name " + source + " does not exist");
    }
    auto& event = events[source];
    if(!event.ongoing) {
        throw std::runtime_error("Event with name " + source + " has not been started");
    }
    event.duration = std::chrono::duration_cast<std::chrono::microseconds>(now - event.timestamp);
    event.ongoing = false;

    PipelineEvent pipelineEvent;
    pipelineEvent.nodeId = nodeId;
    pipelineEvent.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(event.timestamp.time_since_epoch()).count();
    pipelineEvent.duration = event.duration.count();
    pipelineEvent.type = event.type;
    pipelineEvent.source = source;
    pipelineEvent.sequenceNum = sequenceNum++;
    pipelineEvent.setTimestampDevice(std::chrono::steady_clock::now());
    pipelineEvent.ts = pipelineEvent.tsDevice;

    if(out) {
        out->send(std::make_shared<dai::PipelineEvent>(pipelineEvent));
    }

    event.metadata = std::nullopt;
}
void PipelineEventDispatcher::pingEvent(const std::string& source) {
    checkNodeId();
    auto now = std::chrono::steady_clock::now();

    if(events.find(source) == events.end()) {
        throw std::runtime_error("Event with name " + source + " does not exist");
    }
    auto& event = events[source];
    if(event.ongoing) {
        event.duration = std::chrono::duration_cast<std::chrono::microseconds>(now - event.timestamp);
        event.timestamp = now;

        PipelineEvent pipelineEvent;
        pipelineEvent.nodeId = nodeId;
        pipelineEvent.duration = event.duration.count();
        pipelineEvent.type = event.type;
        pipelineEvent.source = source;
        pipelineEvent.metadata = std::nullopt;

        if(out) {
            out->send(std::make_shared<dai::PipelineEvent>(pipelineEvent));
        }
    } else {
        event.timestamp = now;
        event.ongoing = true;
    }
}

}  // namespace utility
}  // namespace dai
