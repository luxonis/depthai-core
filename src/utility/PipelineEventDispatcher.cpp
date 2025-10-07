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
void PipelineEventDispatcher::addEvent(const std::string& source, PipelineEvent::Type type) {
    if(!source.empty()) {
        if(events.find(source) != events.end()) {
            throw std::runtime_error("Event with name '" + source + "' already exists");
        }
        PipelineEvent event;
        event.type = type;
        event.source = source;
        events[source] = {event, false};
    }
}
void PipelineEventDispatcher::startEvent(const std::string& source, std::optional<uint32_t> queueSize, std::optional<Buffer> metadata) {
    // TODO add mutex
    checkNodeId();
    if(events.find(source) == events.end()) {
        throw std::runtime_error("Event with name " + source + " does not exist");
    }
    auto& event = events[source];
    if(event.ongoing) {
        throw std::runtime_error("Event with name " + source + " is already ongoing");
    }
    event.event.setTimestamp(std::chrono::steady_clock::now());
    event.event.tsDevice = event.event.ts;
    ++event.event.sequenceNum;
    event.event.nodeId = nodeId;
    event.event.metadata = std::move(metadata);
    event.event.queueSize = std::move(queueSize);
    event.event.interval = PipelineEvent::Interval::START;
    // type and source are already set
    event.ongoing = true;

    if(out) {
        out->send(std::make_shared<dai::PipelineEvent>(event.event));
    }
}
void PipelineEventDispatcher::endEvent(const std::string& source, std::optional<uint32_t> queueSize, std::optional<Buffer> metadata) {
    // TODO add mutex
    checkNodeId();
    auto now = std::chrono::steady_clock::now();

    if(events.find(source) == events.end()) {
        throw std::runtime_error("Event with name " + source + " does not exist");
    }
    auto& event = events[source];
    if(!event.ongoing) {
        throw std::runtime_error("Event with name " + source + " has not been started");
    }

    event.event.setTimestamp(now);
    event.event.tsDevice = event.event.ts;
    event.event.nodeId = nodeId;
    event.event.metadata = std::move(metadata);
    event.event.queueSize = std::move(queueSize);
    event.event.interval = PipelineEvent::Interval::END;
    // type and source are already set
    event.ongoing = false;

    if(out) {
        out->send(std::make_shared<dai::PipelineEvent>(event.event));
    }

    event.event.metadata = std::nullopt;
    event.event.queueSize = std::nullopt;
}
void PipelineEventDispatcher::pingEvent(const std::string& source) {
    // TODO add mutex
    checkNodeId();
    auto now = std::chrono::steady_clock::now();

    if(events.find(source) == events.end()) {
        throw std::runtime_error("Event with name " + source + " does not exist");
    }
    auto& event = events[source];
    if(event.ongoing) {
        throw std::runtime_error("Event with name " + source + " is already ongoing");
    }
    event.event.setTimestamp(now);
    event.event.tsDevice = event.event.ts;
    ++event.event.sequenceNum;
    event.event.nodeId = nodeId;
    event.event.interval = PipelineEvent::Interval::NONE;
    // type and source are already set

    if(out) {
        out->send(std::make_shared<dai::PipelineEvent>(event.event));
    }
}

}  // namespace utility
}  // namespace dai
