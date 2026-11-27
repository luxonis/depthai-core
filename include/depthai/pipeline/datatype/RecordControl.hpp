#pragma once

#include <cstdint>
#include <memory>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Starts or stops a RecordVideo segment.
 *
 * A value below 0 in maxFrames, maxDurationMs, or maxSizeKb means that limit is off.
 * A START with 0 in any of those fields is ignored. STOP ignores all three.
 * Reaching a limit stops the segment. Recording stays idle until the next START.
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
    /**
     * Written frame payload, in units of 1024 bytes. Below 0 means no limit.
     * The frame that reaches the limit is written, then the segment stops.
     * Container and metadata bytes are not counted.
     */
    std::int32_t maxSizeKb = -1;

    RecordControl() = default;
    ~RecordControl() override;

    /**
     * START with no frame limit, no duration limit, and no size limit.
     */
    static std::shared_ptr<RecordControl> start();
    /**
     * START that stops the segment when either limit is reached. Below 0 means that limit is off.
     * maxSizeKb stays off.
     */
    static std::shared_ptr<RecordControl> start(std::int32_t maxFrames, std::int32_t maxDurationMs);
    /**
     * START that stops the segment when any limit is reached. Below 0 means that limit is off.
     * maxSizeKb is 1024-byte units of written frame payload.
     */
    static std::shared_ptr<RecordControl> start(std::int32_t maxFrames, std::int32_t maxDurationMs, std::int32_t maxSizeKb);
    /**
     * STOP the current segment. maxFrames, maxDurationMs, and maxSizeKb are ignored.
     */
    static std::shared_ptr<RecordControl> stop();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::RecordControl;
    }

    DEPTHAI_SERIALIZE(RecordControl, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, command, maxFrames, maxDurationMs, maxSizeKb);
};

}  // namespace dai
