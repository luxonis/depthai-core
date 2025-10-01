#pragma once

#include <vector>

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
    };
    enum class Interval : std::int32_t {
        NONE = 0,
        START = 1,
        END = 2
    };

    PipelineEvent() = default;
    virtual ~PipelineEvent() = default;

    int64_t nodeId = -1;
    Buffer metadata;
    Interval interval = Interval::NONE;
    Type type = Type::CUSTOM;
    std::string source;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineEvent;
    };

    DEPTHAI_SERIALIZE(PipelineEvent, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodeId, metadata, interval, type, source);
};

}  // namespace dai
