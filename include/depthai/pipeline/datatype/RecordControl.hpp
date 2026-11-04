#pragma once

#include <cstdint>
#include <memory>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Starts or stops a RecordVideo segment.
 *
 * A value below 0 in maxFrames or maxDurationMs means that limit is off.
 * A START with 0 in either field is ignored. STOP ignores both fields.
 */
class RecordControl : public Buffer {
   public:
    enum class Command : std::int32_t { START = 0, STOP = 1 };

    /** START records and STOP ends the current segment. */
    Command command = Command::START;
    /** Frames to write in the segment, including pre-trigger frames. Below 0 means no limit. */
    std::int32_t maxFrames = -1;
    /** Milliseconds from the moment START is applied. Below 0 means no limit. */
    std::int32_t maxDurationMs = -1;

    RecordControl() = default;
    ~RecordControl() override;

    /**
     * START with no frame limit and no duration limit.
     */
    static std::shared_ptr<RecordControl> start();
    /**
     * START that closes the segment when either limit is reached. Below 0 means that limit is off.
     */
    static std::shared_ptr<RecordControl> start(std::int32_t maxFrames, std::int32_t maxDurationMs);
    /**
     * STOP the current segment. maxFrames and maxDurationMs are ignored.
     */
    static std::shared_ptr<RecordControl> stop();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::RecordControl;
    }

    DEPTHAI_SERIALIZE(RecordControl, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, command, maxFrames, maxDurationMs);
};

}  // namespace dai
