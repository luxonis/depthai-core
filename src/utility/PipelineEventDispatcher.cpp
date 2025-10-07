#include "depthai/utility/PipelineEventDispatcher.hpp"

#include <optional>

namespace dai {
namespace utility {

std::string typeToString(PipelineEvent::Type type) {
    switch(type) {
        case PipelineEvent::Type::CUSTOM:
            return "CUSTOM";
        case PipelineEvent::Type::LOOP:
            return "LOOP";
        case PipelineEvent::Type::INPUT:
            return "INPUT";
        case PipelineEvent::Type::OUTPUT:
            return "OUTPUT";
        case PipelineEvent::Type::INPUT_BLOCK:
            return "INPUT_BLOCK";
        case PipelineEvent::Type::OUTPUT_BLOCK:
            return "OUTPUT_BLOCK";
        default:
            return "UNKNOWN";
    }
}

std::string makeKey(PipelineEvent::Type type, const std::string& source) {
    return typeToString(type) + "#" + source;
}

void PipelineEventDispatcher::checkNodeId() {
    if(nodeId == -1) {
        throw std::runtime_error("Node ID not set on PipelineEventDispatcher");
    }
}
void PipelineEventDispatcher::setNodeId(int64_t id) {
    nodeId = id;
}
void PipelineEventDispatcher::startEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize) {
    // TODO add mutex
    checkNodeId();
    auto& event = events[makeKey(type, source)];
    if(event.ongoing) {
        throw std::runtime_error("Event with name " + source + " is already ongoing");
    }
    event.event.setTimestamp(std::chrono::steady_clock::now());
    event.event.tsDevice = event.event.ts;
    ++event.event.sequenceNum;
    event.event.nodeId = nodeId;
    event.event.queueSize = std::move(queueSize);
    event.event.interval = PipelineEvent::Interval::START;
    // type and source are already set
    event.ongoing = true;

    if(out) {
        out->send(std::make_shared<dai::PipelineEvent>(event));
    }
}
void PipelineEventDispatcher::startInputEvent(const std::string& source, std::optional<uint32_t> queueSize) {
    startEvent(PipelineEvent::Type::INPUT, source, std::move(queueSize));
}
void PipelineEventDispatcher::startOutputEvent(const std::string& source) {
    startEvent(PipelineEvent::Type::OUTPUT, source, std::nullopt);
}
void PipelineEventDispatcher::startCustomEvent(const std::string& source) {
    startEvent(PipelineEvent::Type::CUSTOM, source, std::nullopt);
}
void PipelineEventDispatcher::endEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize) {
    // TODO add mutex
    checkNodeId();
    auto now = std::chrono::steady_clock::now();

    auto& event = events[makeKey(type, source)];
    if(!event.ongoing) {
        throw std::runtime_error("Event with name " + source + " has not been started");
    }

    event.event.setTimestamp(now);
    event.event.tsDevice = event.event.ts;
    event.event.nodeId = nodeId;
    event.event.queueSize = std::move(queueSize);
    event.event.interval = PipelineEvent::Interval::END;
    // type and source are already set
    event.ongoing = false;

    if(out) {
        out->send(std::make_shared<dai::PipelineEvent>(event.event));
    }

    event.event.queueSize = std::nullopt;
}
void PipelineEventDispatcher::endInputEvent(const std::string& source, std::optional<uint32_t> queueSize) {
    endEvent(PipelineEvent::Type::INPUT, source, std::move(queueSize));
}
void PipelineEventDispatcher::endOutputEvent(const std::string& source) {
    endEvent(PipelineEvent::Type::OUTPUT, source, std::nullopt);
}
void PipelineEventDispatcher::endCustomEvent(const std::string& source) {
    endEvent(PipelineEvent::Type::CUSTOM, source, std::nullopt);
}
void PipelineEventDispatcher::pingEvent(PipelineEvent::Type type, const std::string& source) {
    // TODO add mutex
    checkNodeId();
    auto now = std::chrono::steady_clock::now();

    auto& event = events[makeKey(type, source)];
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
void PipelineEventDispatcher::pingMainLoopEvent() {
    pingEvent(PipelineEvent::Type::LOOP, "_mainLoop");
}
void PipelineEventDispatcher::pingCustomEvent(const std::string& source) {
    pingEvent(PipelineEvent::Type::CUSTOM, source);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::blockEvent(PipelineEvent::Type type, const std::string& source) {
    return BlockPipelineEvent(*this, type, source);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::inputBlockEvent(const std::string& source) {
    // For convenience due to the default source
    return blockEvent(PipelineEvent::Type::INPUT_BLOCK, source);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::outputBlockEvent(const std::string& source) {
    // For convenience due to the default source
    return blockEvent(PipelineEvent::Type::OUTPUT_BLOCK, source);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::customBlockEvent(const std::string& source) {
    // For convenience due to the default source
    return blockEvent(PipelineEvent::Type::CUSTOM, source);
}

}  // namespace utility
}  // namespace dai
