#include "depthai/pipeline/datatype/TraceEvents.hpp"

namespace dai {

QueueTraceEvent::Serialized QueueTraceEvent::serialize() const {
    return {data, raw};
}

QueueTraceEvent::QueueTraceEvent() : Buffer(std::make_shared<RawQueueTraceEvent>()), event(*dynamic_cast<RawQueueTraceEvent*>(raw.get())) {}
QueueTraceEvent::QueueTraceEvent(std::shared_ptr<RawQueueTraceEvent> ptr) : Buffer(std::move(ptr)), event(*dynamic_cast<RawQueueTraceEvent*>(raw.get())) {}

dai::RawQueueTraceEvent QueueTraceEvent::get() const {
    return event;
}

QueueTraceEvent& QueueTraceEvent::set(dai::RawQueueTraceEvent event) {
    this->event = event;
    return *this;
}

NodeTraceEvent::Serialized NodeTraceEvent::serialize() const {
    return {data, raw};
}

NodeTraceEvent::NodeTraceEvent() : Buffer(std::make_shared<RawNodeTraceEvent>()), event(*dynamic_cast<RawNodeTraceEvent*>(raw.get())) {}
NodeTraceEvent::NodeTraceEvent(std::shared_ptr<RawNodeTraceEvent> ptr) : Buffer(std::move(ptr)), event(*dynamic_cast<RawNodeTraceEvent*>(raw.get())) {}

dai::RawNodeTraceEvent NodeTraceEvent::get() const {
    return event;
}

unsigned int NodeTraceEvent::getNodeId() const {
    return event.nodeId;
}

NodeTraceEvent& NodeTraceEvent::set(dai::RawNodeTraceEvent event) {
    this->event = event;
    return *this;
}

void NodeTraceEvent::setNodeId(unsigned int nodeId) {
    event.nodeId = nodeId;
}

void NodeTraceEvent::setLoopStart(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    event.timeLoopStart.set(tp);
}

void NodeTraceEvent::setProcStart(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    event.timeProcStart.set(tp);
}

void NodeTraceEvent::setProcEnd(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    event.timeProcEnd.set(tp);
}

void NodeTraceEvent::setLoopEnd(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    event.timeLoopEnd.set(tp);
}

std::chrono::duration<int64_t, std::nano> NodeTraceEvent::getTimeToGetMessages() const {
    return event.timeProcStart.get() - event.timeLoopStart.get();
}

std::chrono::duration<int64_t, std::nano> NodeTraceEvent::getTimeToProcess() const {
    return event.timeProcEnd.get() - event.timeProcStart.get();
}

std::chrono::duration<int64_t, std::nano> NodeTraceEvent::getTimeToSendMessages() const {
    return event.timeLoopEnd.get() - event.timeProcEnd.get();
}

std::chrono::duration<int64_t, std::nano> NodeTraceEvent::getTotalTime() const {
    return event.timeLoopEnd.get() - event.timeLoopStart.get();
}

}  // namespace dai
