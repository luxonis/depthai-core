#pragma once

#include <cstdint>
#include <string>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * State of a RecordVideo node at a transition.
 *
 * sequenceNum counts status messages of this node from 0. Timestamps are the
 * time of the transition on the processor clock.
 */
class RecordStatus : public Buffer {
   public:
    enum class State : std::int32_t { STARTED = 0, STOPPED = 1, ERROR = 2 };

    /** STARTED, STOPPED, or ERROR. */
    State state = State::STOPPED;
    /** Segment index from 0, or -1 when no segment has been opened. */
    std::int32_t segment = -1;
    /** Frames written in this segment, including pre-trigger frames. */
    std::int32_t frameCount = 0;
    /** sequenceNum of the first written frame, or -1 when frameCount is 0. */
    std::int64_t firstSequenceNum = -1;
    /** sequenceNum of the last written frame, or -1 when frameCount is 0. */
    std::int64_t lastSequenceNum = -1;
    /** Frames skipped in this segment, from gaps in written sequence numbers. */
    std::int32_t skippedFrames = 0;
    /** Processor path of the recording: host outputDirectory/name, or the device store path. */
    std::string path;
    /** Empty unless state is ERROR. */
    std::string error;

    RecordStatus() = default;
    ~RecordStatus() override;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = this->getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::RecordStatus;
    }

    DEPTHAI_SERIALIZE(RecordStatus,
                      Buffer::sequenceNum,
                      Buffer::ts,
                      Buffer::tsDevice,
                      Buffer::tsSystem,
                      state,
                      segment,
                      frameCount,
                      firstSequenceNum,
                      lastSequenceNum,
                      skippedFrames,
                      path,
                      error);
};

}  // namespace dai
