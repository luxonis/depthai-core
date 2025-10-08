#pragma once

#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "depthai/common/optional.hpp"

namespace dai {

class NodeState {
   public:
    struct DurationEvent {
        PipelineEvent startEvent;
        uint64_t durationUs;
        float fps;
        DEPTHAI_SERIALIZE(DurationEvent, startEvent, durationUs, fps);
    };
    struct TimingStats {
        uint64_t minMicros = -1;
        uint64_t maxMicros;
        uint64_t averageMicrosRecent;
        uint64_t stdDevMicrosRecent;
        uint64_t minMicrosRecent = -1;
        uint64_t maxMicrosRecent;
        uint64_t medianMicrosRecent;
        DEPTHAI_SERIALIZE(TimingStats, minMicros, maxMicros, averageMicrosRecent, stdDevMicrosRecent, minMicrosRecent, maxMicrosRecent, medianMicrosRecent);
    };
    struct Timing {
        float fps;
        TimingStats durationStats;
        DEPTHAI_SERIALIZE(Timing, fps, durationStats);
    };
    struct QueueStats {
        uint32_t maxQueued;
        uint32_t minQueuedRecent;
        uint32_t maxQueuedRecent;
        uint32_t medianQueuedRecent;
        DEPTHAI_SERIALIZE(QueueStats, maxQueued, minQueuedRecent, maxQueuedRecent, medianQueuedRecent);
    };
    struct InputQueueState {
        enum class State : std::int32_t {
            IDLE = 0,
            WAITING = 1,
            BLOCKED = 2
        } state = State::IDLE;
        uint32_t numQueued;
        Timing timing;
        QueueStats queueStats;
        DEPTHAI_SERIALIZE(InputQueueState, state, numQueued, timing);
    };
    struct OutputQueueState {
        enum class State : std::int32_t {
            IDLE = 0,
            SENDING = 1
        } state = State::IDLE;
        Timing timing;
        DEPTHAI_SERIALIZE(OutputQueueState, state, timing);
    };
    enum class State : std::int32_t {
        IDLE = 0,
        GETTING_INPUTS = 1,
        PROCESSING = 2,
        SENDING_OUTPUTS = 3
    };

    State state = State::IDLE;
    std::vector<DurationEvent> events;
    std::unordered_map<std::string, OutputQueueState> outputStates;
    std::unordered_map<std::string, InputQueueState> inputStates;
    Timing inputsGetTiming;
    Timing outputsSendTiming;
    Timing mainLoopTiming;
    std::unordered_map<std::string, Timing> otherTimings;

    DEPTHAI_SERIALIZE(NodeState, events, outputStates, inputStates, inputsGetTiming, outputsSendTiming, mainLoopTiming, otherTimings);
};

/**
 * Pipeline event message.
 */
class PipelineState : public Buffer {
   public:
    PipelineState() = default;
    virtual ~PipelineState() = default;

    std::unordered_map<int64_t, NodeState> nodeStates;
    uint32_t configSequenceNum = 0;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineState;
    };

    DEPTHAI_SERIALIZE(PipelineState, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodeStates, configSequenceNum);
};

}  // namespace dai
