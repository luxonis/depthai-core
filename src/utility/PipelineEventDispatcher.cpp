#include "depthai/utility/PipelineEventDispatcher.hpp"

#include <optional>

namespace dai {
namespace utility {

constexpr const char* OUTPUT_BLOCK_NAME = "sendOutputs";
constexpr const char* INPUT_BLOCK_NAME = "getInputs";

PipelineEventDispatcherInterface::~PipelineEventDispatcherInterface() = default;

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

bool blacklist(PipelineEvent::Type type, const std::string& source) {
    if(type == PipelineEvent::Type::OUTPUT && source == "pipelineEventOutput") return true;
    return false;
}

void PipelineEventDispatcher::checkNodeId() {
    if(nodeId == -1) {
        throw std::runtime_error("Node ID not set on PipelineEventDispatcher");
    }
}
void PipelineEventDispatcher::startEvent(PipelineEvent::Type type, const std::string& source, std::optional<uint32_t> queueSize) {
    if(!sendEvents) return;
    if(blacklist(type, source)) return;
    checkNodeId();

    std::lock_guard<std::mutex> lock(mutex);

    auto& event = events[makeKey(type, source)];
    event.event.setTimestamp(std::chrono::steady_clock::now());
    event.event.tsDevice = event.event.ts;
    ++event.event.sequenceNum;
    event.event.nodeId = nodeId;
    event.event.queueSize = std::move(queueSize);
    event.event.interval = PipelineEvent::Interval::START;
    event.event.type = type;
    event.event.source = source;
    event.ongoing = true;

    if(out) try {
            out->send(std::make_shared<dai::PipelineEvent>(event.event));
        } catch(const MessageQueue::QueueException&) {
        }
    else throw std::runtime_error("PipelineEventDispatcher output is null");
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
    if(!sendEvents) return;
    if(blacklist(type, source)) return;
    checkNodeId();

    std::lock_guard<std::mutex> lock(mutex);

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
    event.event.type = type;
    event.event.source = source;
    event.ongoing = false;

    if(out) try {
            out->send(std::make_shared<dai::PipelineEvent>(event.event));
        } catch(const MessageQueue::QueueException&) {
        }
    else throw std::runtime_error("PipelineEventDispatcher output is null");

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
void PipelineEventDispatcher::startTrackedEvent(PipelineEvent event, std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts) {
    if(!sendEvents) return;
    if(blacklist(event.type, event.source)) return;
    checkNodeId();

    std::lock_guard<std::mutex> lock(mutex);

    if(!ts.has_value())
        event.setTimestamp(std::chrono::steady_clock::now());
    else
        event.setTimestamp(ts.value());
    event.tsDevice = event.ts;
    event.nodeId = nodeId;
    event.interval = PipelineEvent::Interval::START;

    if(out) try {
            out->send(std::make_shared<dai::PipelineEvent>(event));
        } catch(const MessageQueue::QueueException&) {
        }
    else throw std::runtime_error("PipelineEventDispatcher output is null");
}
void PipelineEventDispatcher::startTrackedEvent(PipelineEvent::Type type,
                                                const std::string& source,
                                                int64_t sequenceNum,
                                                std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts) {
    PipelineEvent event;
    event.type = type;
    event.source = source;
    event.sequenceNum = sequenceNum;
    startTrackedEvent(event, ts);
}
void PipelineEventDispatcher::endTrackedEvent(PipelineEvent event, std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts) {
    if(!sendEvents) return;
    if(blacklist(event.type, event.source)) return;
    checkNodeId();

    std::lock_guard<std::mutex> lock(mutex);

    if(!ts.has_value())
        event.setTimestamp(std::chrono::steady_clock::now());
    else
        event.setTimestamp(ts.value());
    event.tsDevice = event.ts;
    event.nodeId = nodeId;
    event.interval = PipelineEvent::Interval::END;

    if(out) try {
            out->send(std::make_shared<dai::PipelineEvent>(event));
        } catch(const MessageQueue::QueueException&) {
        }
    else throw std::runtime_error("PipelineEventDispatcher output is null");
}
void PipelineEventDispatcher::endTrackedEvent(PipelineEvent::Type type,
                                              const std::string& source,
                                              int64_t sequenceNum,
                                              std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts) {
    PipelineEvent event;
    event.type = type;
    event.source = source;
    event.sequenceNum = sequenceNum;
    endTrackedEvent(event, ts);
}
void PipelineEventDispatcher::pingEvent(PipelineEvent::Type type, const std::string& source) {
    if(!sendEvents) return;
    if(blacklist(type, source)) return;
    checkNodeId();

    std::lock_guard<std::mutex> lock(mutex);

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
    event.event.type = type;
    event.event.source = source;

    if(out) try {
            out->send(std::make_shared<dai::PipelineEvent>(event.event));
        } catch(const MessageQueue::QueueException&) {
        }
    else throw std::runtime_error("PipelineEventDispatcher output is null");
}
void PipelineEventDispatcher::pingMainLoopEvent() {
    pingEvent(PipelineEvent::Type::LOOP, "_mainLoop");
}
void PipelineEventDispatcher::pingCustomEvent(const std::string& source) {
    pingEvent(PipelineEvent::Type::CUSTOM, source);
}
void PipelineEventDispatcher::pingInputEvent(const std::string& source, PipelineEvent::Status status, std::optional<uint32_t> queueSize) {
    if(!sendEvents) return;
    if(blacklist(PipelineEvent::Type::INPUT, source)) return;
    checkNodeId();

    std::lock_guard<std::mutex> lock(mutex);

    auto now = std::chrono::steady_clock::now();

    auto& event = events[makeKey(PipelineEvent::Type::INPUT, source)];
    PipelineEvent eventCopy = event.event;
    eventCopy.setTimestamp(now);
    eventCopy.tsDevice = eventCopy.ts;
    eventCopy.nodeId = nodeId;
    eventCopy.status = std::move(status);
    eventCopy.queueSize = std::move(queueSize);
    eventCopy.interval = PipelineEvent::Interval::NONE;
    eventCopy.type = PipelineEvent::Type::INPUT;
    eventCopy.source = source;

    if(out) try {
            out->send(std::make_shared<dai::PipelineEvent>(eventCopy));
        } catch(const MessageQueue::QueueException&) {
        }
    else throw std::runtime_error("PipelineEventDispatcher output is null");
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::blockEvent(PipelineEvent::Type type,
                                                                                const std::string& source,
                                                                                std::optional<std::chrono::time_point<std::chrono::steady_clock>> ts) {
    return BlockPipelineEvent(*this, type, source, ts);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::inputBlockEvent() {
    // For convenience due to the default source
    return blockEvent(PipelineEvent::Type::INPUT_BLOCK, INPUT_BLOCK_NAME);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::outputBlockEvent() {
    // For convenience due to the default source
    return blockEvent(PipelineEvent::Type::OUTPUT_BLOCK, OUTPUT_BLOCK_NAME);
}
PipelineEventDispatcher::BlockPipelineEvent PipelineEventDispatcher::customBlockEvent(const std::string& source) {
    // For convenience due to the default source
    return blockEvent(PipelineEvent::Type::CUSTOM, source);
}

}  // namespace utility
}  // namespace dai
