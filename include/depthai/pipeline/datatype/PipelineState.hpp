#pragma once

#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {

class NodeState {
   public:
    struct Timing {
        uint64_t averageMicros;
        uint64_t stdDevMicros;
        DEPTHAI_SERIALIZE(Timing, averageMicros, stdDevMicros);
    };
    std::vector<PipelineEvent> events;
    std::unordered_map<PipelineEvent::EventType, Timing> timingsByType;
    std::unordered_map<std::string, Timing> timingsByInstance;

    DEPTHAI_SERIALIZE(NodeState, events, timingsByType, timingsByInstance);
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
