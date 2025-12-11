#pragma once
#include <array>
#include <cstdint>
#include <vector>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * TrackedFeature structure
 *
 */
struct TrackedFeature {
    /**
     *  x, y position of the detected feature
     */
    Point2f position;

    /**
     *  Feature ID. Persistent between frames if motion estimation is enabled.
     */
    uint32_t id = 0;

    /**
     *  Feature age in frames
     */
    uint32_t age = 0;

    /**
     *  Feature harris score
     */
    float harrisScore = 0.f;

    /**
     *  Feature tracking error
     */
    float trackingError = 0.f;

    /**
     *  Feature descriptor
     */
    std::array<uint8_t, 32> descriptor = {0};
};
DEPTHAI_SERIALIZE_EXT(TrackedFeature, position, id, age, harrisScore, trackingError, descriptor);

/**
 * TrackedFeatures message. Carries position (X, Y) of tracked features and their ID.
 */
class TrackedFeatures : public Buffer {
   public:
    /**
     * Construct TrackedFeatures message.
     */
    TrackedFeatures() = default;
    virtual ~TrackedFeatures();

    std::vector<TrackedFeature> trackedFeatures;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::TrackedFeatures;
    }

    #ifndef DEPTHAI_MESSAGES_RVC2
    DEPTHAI_SERIALIZE(TrackedFeatures, trackedFeatures, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem);
    #else
    DEPTHAI_SERIALIZE(TrackedFeatures, trackedFeatures, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice);
    #endif
};

}  // namespace dai
