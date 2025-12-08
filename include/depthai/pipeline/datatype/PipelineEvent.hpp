#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Pipeline event message.
 */
class PipelineEvent : public Buffer {
   public:
    enum class Type : std::int32_t {
        CUSTOM = 0,
        LOOP = 1,
        INPUT = 2,
        OUTPUT = 3,
        INPUT_BLOCK = 4,
        OUTPUT_BLOCK = 5,
    };
    enum class Interval : std::int32_t { NONE = 0, START = 1, END = 2 };
    enum class Status : std::int32_t {
        OK = 0,
        BLOCKED = -1,
        CANCELLED = -2,
    };

    PipelineEvent() = default;
    virtual ~PipelineEvent();

    int64_t nodeId = -1;
    Status status = Status::OK;  // Used for input status reporting (if it is blocked)
    std::optional<uint32_t> queueSize;
    Interval interval = Interval::NONE;
    Type type = Type::CUSTOM;
    std::string source;  // e.g., input/output/custom event name - used in conjunction with type to identify the event

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineEvent;
    };

    DEPTHAI_SERIALIZE(PipelineEvent, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodeId, status, queueSize, interval, type, source);
};

}  // namespace dai
