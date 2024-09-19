#pragma once

#include <vector>

// project
#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai/common/Point3f.hpp"

namespace dai {

/**
 * Keypoints message. Carries keypoints data.
 */
 class Keypoints : public Buffer {
   public:
    /**
     * Construct Keypoints message.
     */
    Keypoints() = default;
    virtual ~Keypoints() = default;

    /// Keypoints
    std::vector<Point3f> keypoints;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Keypoints;
    };

    // getters
    const std::vector<Point3f>& getKeypoints() const;

    // setters
    Keypoints& setKeypoints(const std::vector<Point3f>& points);

    DEPTHAI_SERIALIZE(Keypoints, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, keypoints);
};

}  // namespace dai
