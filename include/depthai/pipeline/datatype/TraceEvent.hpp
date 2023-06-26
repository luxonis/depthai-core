#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTraceEvent.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * ToFConfig message. Carries config for feature tracking algorithm
 */
class TraceEvent : public Buffer {
    Serialized serialize() const override;
    RawTraceEvent& event;

   public:
    /**
     * Construct TraceEvent message.
     */
    TraceEvent();
    explicit TraceEvent(std::shared_ptr<RawTraceEvent> ptr);
    virtual ~TraceEvent() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
     dai::TraceEvent& set(dai::RawTraceEvent event);

    /**
     * Retrieve the raw event.
     * @returns the raw event
     */
    dai::RawTraceEvent get() const;
};

}  // namespace dai