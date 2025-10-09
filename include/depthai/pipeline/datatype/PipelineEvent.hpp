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
    enum class Interval : std::int32_t {
        NONE = 0,
        START = 1,
        END = 2
    };

    PipelineEvent() = default;
    virtual ~PipelineEvent() = default;

    int64_t nodeId = -1;
    int32_t status = 0;
    std::optional<uint32_t> queueSize;
    Interval interval = Interval::NONE;
    Type type = Type::CUSTOM;
    std::string source;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineEvent;
    };

    DEPTHAI_SERIALIZE(PipelineEvent, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodeId, status, queueSize, interval, type, source);
};

}  // namespace dai
