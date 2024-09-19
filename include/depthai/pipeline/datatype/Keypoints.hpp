#pragma once

#include <vector>

// project
#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai/common/Point2f.hpp"
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
    ~Keypoints() override = default;

    /// Keypoints
    std::vector<Point3f> keypoints;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Keypoints;
    };

    // getters
    const std::vector<Point3f>& getKeypoints() const;

    // setters

    /**
     * Set 3D keypoints
     *
     * @param keypoints detected 3D keypoints
     * @param scores confidence scores for each keypoint
     * @param confidence_threshold confidence threshold
     *  
     * @returns keypoints message
     */
    Keypoints& setKeypoints(const std::vector<Point3f>& keypoints);
    Keypoints& setKeypoints(const std::vector<Point3f>& keypoints, const std::vector<float>& scores, float confidenceThreshold);

    /**
     * Set 2D keypoints
     *
     * @param keypoints detected 2D keypoints
     * @param scores confidence scores for each keypoint
     * @param confidence_threshold confidence threshold
     *  
     * @returns keypoints message
     */
    Keypoints& setKeypoints(const std::vector<Point2f>& keypoints);
    Keypoints& setKeypoints(const std::vector<Point2f>& keypoints, const std::vector<float>& scores, float confidenceThreshold);

    DEPTHAI_SERIALIZE(Keypoints, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, keypoints);
};

}  // namespace dai
