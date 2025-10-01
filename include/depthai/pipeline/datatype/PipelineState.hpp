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
        DEPTHAI_SERIALIZE(DurationEvent, startEvent, durationUs);
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
    struct QueueStats {
        uint32_t maxQueued;
        uint32_t minQueuedRecent;
        uint32_t maxQueuedRecent;
        uint32_t medianQueuedRecent;
        DEPTHAI_SERIALIZE(QueueStats, maxQueued, minQueuedRecent, maxQueuedRecent, medianQueuedRecent);
    };
    struct QueueState {
        bool waiting;
        uint32_t numQueued;
        TimingStats timingStats;
        QueueStats queueStats;
        DEPTHAI_SERIALIZE(QueueState, waiting, numQueued, timingStats);
    };
    std::vector<DurationEvent> events;
    std::unordered_map<PipelineEvent::Type, TimingStats> timingsByType;
    std::unordered_map<std::string, QueueState> inputStates;
    std::unordered_map<std::string, QueueState> outputStates;
    TimingStats mainLoopStats;
    std::unordered_map<std::string, TimingStats> otherStats;

    DEPTHAI_SERIALIZE(NodeState, events, timingsByType, inputStates, outputStates, mainLoopStats, otherStats);
};

/**
 * Pipeline event message.
 */
class PipelineState : public Buffer {
   public:
    PipelineState() = default;
    virtual ~PipelineState() = default;

    std::unordered_map<int64_t, NodeState> nodeStates;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineState;
    };

    DEPTHAI_SERIALIZE(PipelineState, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodeStates);
};

}  // namespace dai
