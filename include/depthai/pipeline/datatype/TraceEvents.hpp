#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTraceEvents.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * QueueTraceEvent message. Carries config for feature tracking algorithm
 */
class QueueTraceEvent : public Buffer {
    Serialized serialize() const override;
    RawQueueTraceEvent& event;

   public:
    /**
     * Construct QueueTraceEvent message.
     */
    QueueTraceEvent();
    explicit QueueTraceEvent(std::shared_ptr<RawQueueTraceEvent> ptr);
    virtual ~QueueTraceEvent() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    dai::QueueTraceEvent& set(dai::RawQueueTraceEvent event);

    /**
     * Retrieve the raw event.
     * @returns the raw event
     */
    dai::RawQueueTraceEvent get() const;
};

/**
 * QueueTraceEvent message. Carries config for feature tracking algorithm
 */
class NodeTraceEvent : public Buffer {
    Serialized serialize() const override;
    RawNodeTraceEvent& event;

   public:
    /**
     * Construct QueueTraceEvent message.
     */
    NodeTraceEvent();
    explicit NodeTraceEvent(std::shared_ptr<RawNodeTraceEvent> ptr);
    virtual ~NodeTraceEvent() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    dai::NodeTraceEvent& set(dai::RawNodeTraceEvent event);

    /**
     * Retrieve the raw event.
     * @returns the raw event
     */
    dai::RawNodeTraceEvent get() const;

    /**
     * Get node ID
     * @returns node ID
     */
    unsigned int getNodeId() const;

    /**
     * Set start of the loop time
     * @param tp start of the loop time
     */
    void setLoopStart(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);

    /**
     * Set proc start time
     * @param tp proc start time
     */
    void setProcStart(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);

    /**
     * Set proc end time
     * @param tp proc end time
     */
    void setProcEnd(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);

    /**
     * Set loop end time
     * @param tp loop end time
     */
    void setLoopEnd(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp);

    /**
     * Set node ID
     * @param id node ID
     */
    void setNodeId(unsigned int id);

    /**
     * Get time taken to get the images from the previous node
     * @returns time taken to get the images from the previous node
     */
    std::chrono::duration<int64_t, std::nano> getTimeToGetMessages() const;

    /**
     * Get time taken to process the message
     * @returns time taken to process the message
     */
    std::chrono::duration<int64_t, std::nano> getTimeToProcess() const;

    /**
     * Get time taken to send the message to the next node
     * @returns time taken to send the message to the next node
     */
    std::chrono::duration<int64_t, std::nano> getTimeToSendMessages() const;

    /**
     * Get total time for the node run loop (getting messages + processing + sending messages out)
     * @returns total time for the node run loop (getting messages + processing + sending messages out)
     */
    std::chrono::duration<int64_t, std::nano> getTotalTime() const;
};

}  // namespace dai