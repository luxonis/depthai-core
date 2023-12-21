#pragma once

#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Timestamp.hpp"
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
};
DEPTHAI_SERIALIZE_EXT(TrackedFeature, position, id, age, harrisScore, trackingError);

/**
 * TrackedFeatures message. Carries position (X, Y) of tracked features and their ID.
 */
class TrackedFeatures : public Buffer {
   public:
    /**
     * Construct TrackedFeatures message.
     */
    TrackedFeatures() = default;
    virtual ~TrackedFeatures() = default;

    std::vector<TrackedFeature> trackedFeatures;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::TrackedFeatures;
    };

    DEPTHAI_SERIALIZE(TrackedFeatures, trackedFeatures, sequenceNum, ts, tsDevice);
};

}  // namespace dai
