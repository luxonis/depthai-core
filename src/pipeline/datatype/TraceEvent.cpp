#include "depthai/pipeline/datatype/TraceEvent.hpp"

namespace dai {

TraceEvent::Serialized TraceEvent::serialize() const {
    return {data, raw};
}

TraceEvent::TraceEvent() : Buffer(std::make_shared<RawTraceEvent>()), event(*dynamic_cast<RawTraceEvent*>(raw.get())) {}
TraceEvent::TraceEvent(std::shared_ptr<RawTraceEvent> ptr) : Buffer(std::move(ptr)), event(*dynamic_cast<RawTraceEvent*>(raw.get())) {}

dai::RawTraceEvent TraceEvent::get() const {
    return event;
}

TraceEvent& TraceEvent::set(dai::RawTraceEvent event) {
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

NodeTraceEvent& NodeTraceEvent::set(dai::RawNodeTraceEvent event) {
    this->event = event;
    return *this;
}

}  // namespace dai
