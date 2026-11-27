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
        uint64_t durationUs = 0;
        DEPTHAI_SERIALIZE_DECL(DurationEvent, startEvent, durationUs);
    };
    struct DurationStats {
        uint64_t minMicros = -1;
        uint64_t maxMicros = 0;
        uint64_t averageMicrosRecent = 0;
        uint64_t stdDevMicrosRecent = 0;
        uint64_t minMicrosRecent = -1;
        uint64_t maxMicrosRecent = 0;
        uint64_t medianMicrosRecent = 0;
        DEPTHAI_SERIALIZE_DECL(
            DurationStats, minMicros, maxMicros, averageMicrosRecent, stdDevMicrosRecent, minMicrosRecent, maxMicrosRecent, medianMicrosRecent);
    };
    struct Timing {
        float fps = 0.0f;
        DurationStats durationStats;

        bool isValid() const {
            return durationStats.minMicros <= durationStats.maxMicros;
        }

        DEPTHAI_SERIALIZE_DECL(Timing, fps, durationStats);
    };
    struct QueueStats {
        uint32_t maxQueued = 0;
        uint32_t minQueuedRecent = 0;
        uint32_t maxQueuedRecent = 0;
        uint32_t medianQueuedRecent = 0;
        DEPTHAI_SERIALIZE_DECL(QueueStats, maxQueued, minQueuedRecent, maxQueuedRecent, medianQueuedRecent);
    };
    struct InputQueueState {
        enum class State : std::int32_t {
            IDLE = 0,
            WAITING = 1,  // Waiting to receive a message
            BLOCKED = 2   // An output attempted to send to this input, but the input queue was full
        } state = State::IDLE;
        uint32_t numQueued = 0;
        Timing timing;
        QueueStats queueStats;

        bool isValid() const {
            return timing.isValid();
        }

        DEPTHAI_SERIALIZE_DECL(InputQueueState, state, numQueued, timing, queueStats);
    };
    struct OutputQueueState {
        enum class State : std::int32_t { IDLE = 0, SENDING = 1 } state = State::IDLE;  // SENDING lasts while the input queue is full, unless trySend is used
        Timing timing;

        bool isValid() const {
            return timing.isValid();
        }

        DEPTHAI_SERIALIZE_DECL(OutputQueueState, state, timing);
    };
    enum class State : std::int32_t { IDLE = 0, GETTING_INPUTS = 1, PROCESSING = 2, SENDING_OUTPUTS = 3 };

    State state = State::IDLE;
    std::vector<DurationEvent> events;
    std::unordered_map<std::string, OutputQueueState> outputStates;
    std::unordered_map<std::string, InputQueueState> inputStates;
    Timing inputsGetTiming;
    Timing outputsSendTiming;
    Timing mainLoopTiming;  // Processing time + inputs get + outputs send
    std::unordered_map<std::string, Timing> otherTimings;

    DEPTHAI_SERIALIZE_DECL(NodeState, state, events, outputStates, inputStates, inputsGetTiming, outputsSendTiming, mainLoopTiming, otherTimings);
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

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::PipelineState;
    }

    nlohmann::json toJson() const;

    DEPTHAI_SERIALIZE_DECL(PipelineState, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, Buffer::sequenceNum, nodeStates, configSequenceNum);
};

}  // namespace dai
