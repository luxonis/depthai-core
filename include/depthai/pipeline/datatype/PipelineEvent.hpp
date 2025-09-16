#pragma once

#include <vector>

#include "depthai/common/Timestamp.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Pipeline event message.
 */
class PipelineEvent : public Buffer {
   public:
    enum class EventType : std::int32_t {
        CUSTOM = 0,
        LOOP = 1,
        INPUT = 2,
        OUTPUT = 3,
        FUNC_CALL = 4
    };

    PipelineEvent() = default;
    virtual ~PipelineEvent() = default;

    std::optional<Buffer> metadata;
    uint64_t duration {0}; // Duration in microseconds
    EventType type = EventType::CUSTOM;
    std::variant<std::string, std::pair<std::string, std::string>> source;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PipelineEvent;
    };

    DEPTHAI_SERIALIZE(PipelineEvent, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, metadata, duration, type, source);
};

}  // namespace dai
