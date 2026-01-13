#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {

class NodeState {
   public:
    struct DurationEvent {
        PipelineEvent startEvent;
        uint64_t durationUs;
        DEPTHAI_SERIALIZE(DurationEvent, startEvent, durationUs);
    };
    struct DurationStats {
        uint64_t minMicros = -1;
        uint64_t maxMicros = 0;
        uint64_t averageMicrosRecent = 0;
        uint64_t stdDevMicrosRecent = 0;
        uint64_t minMicrosRecent = -1;
        uint64_t maxMicrosRecent = 0;
        uint64_t medianMicrosRecent = 0;
        DEPTHAI_SERIALIZE(DurationStats, minMicros, maxMicros, averageMicrosRecent, stdDevMicrosRecent, minMicrosRecent, maxMicrosRecent, medianMicrosRecent);
    };
    struct Timing {
        float fps = 0.0f;
        DurationStats durationStats;

        /**
         * Return true if timing stats are valid.
         */
        bool isValid() const {
            return durationStats.minMicros <= durationStats.maxMicros;
        }

        DEPTHAI_SERIALIZE(Timing, fps, durationStats);
    };
    struct QueueStats {
        uint32_t maxQueued = 0;
        uint32_t minQueuedRecent = 0;
        uint32_t maxQueuedRecent = 0;
        uint32_t medianQueuedRecent = 0;
        DEPTHAI_SERIALIZE(QueueStats, maxQueued, minQueuedRecent, maxQueuedRecent, medianQueuedRecent);
    };
    struct InputQueueState {
        // Current state of the input queue.
        enum class State : std::int32_t {
            IDLE = 0,
            WAITING = 1,  // Waiting to receive a message
            BLOCKED = 2   // An output attempted to send to this input, but the input queue was full
        } state = State::IDLE;
        // Number of messages currently queued in the input queue
        uint32_t numQueued = 0;
        // Timing info about this input
        Timing timing;
        // Queue usage stats
        QueueStats queueStats;

        /**
         * Return true if timing stats are valid.
         */
        bool isValid() const {
            return timing.isValid();
        }

        DEPTHAI_SERIALIZE(InputQueueState, state, numQueued, timing, queueStats);
    };
    struct OutputQueueState {
        // Current state of the output queue. Send should ideally be instant. This is not the case when the input queue is full.
        // In that case, the state will be SENDING until there is space in the input queue (unless trySend is used).
        enum class State : std::int32_t { IDLE = 0, SENDING = 1 } state = State::IDLE;
        // Timing info about this output
        Timing timing;

        /**
         * Return true if timing stats are valid.
         */
        bool isValid() const {
            return timing.isValid();
        }

        DEPTHAI_SERIALIZE(OutputQueueState, state, timing);
    };
    enum class State : std::int32_t { IDLE = 0, GETTING_INPUTS = 1, PROCESSING = 2, SENDING_OUTPUTS = 3 };

    // Current state of the node - idle only when not running
    State state = State::IDLE;
    // Optional list of recent events
    std::vector<DurationEvent> events;
    // Info about each output
    std::unordered_map<std::string, OutputQueueState> outputStates;
    // Info about each input
    std::unordered_map<std::string, InputQueueState> inputStates;
    // Time spent getting inputs in a loop
    Timing inputsGetTiming;
    // Time spent sending outputs in a loop
    Timing outputsSendTiming;
    // Main node loop timing (processing time + inputs get + outputs send)
    Timing mainLoopTiming;
    // Other timings that the developer of the node decided to add
    std::unordered_map<std::string, Timing> otherTimings;

    DEPTHAI_SERIALIZE(NodeState, state, events, outputStates, inputStates, inputsGetTiming, outputsSendTiming, mainLoopTiming, otherTimings);
};

/**
 * Pipeline event message.
 */
class PipelineState : public Buffer {
   public:
    PipelineState() = default;
    virtual ~PipelineState();

    std::unordered_map<int64_t, NodeState> nodeStates;
    uint32_t configSequenceNum = 0;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineState;
    };

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::PipelineState;
    }

    nlohmann::json toJson() const;

    DEPTHAI_SERIALIZE(PipelineState, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodeStates, configSequenceNum);
};

}  // namespace dai
