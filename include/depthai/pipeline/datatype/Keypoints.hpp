#pragma once

#include <vector>
#include <optional>

// project
#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

namespace dai {

struct Keypoint {
    float x = 0.f;
    float y = 0.f;
    // TODO(NicikD):
    // should be 
    //  std::optional<float> z;
    //  std::optional<float> confidence;
    // but DEPTHAI_SERIALIZE_EXT doesn't support std::optional
    float z = 0.f;
    float confidence = -1.f;
};

DEPTHAI_SERIALIZE_EXT(Keypoint, x, y, z, confidence);

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
    std::vector<Keypoint> keypoints;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Keypoints;
    };

    // getters
    const std::vector<Keypoint>& getKeypoints() const;

    // setters
    Keypoints& setKeypoints(const std::vector<Keypoint>& keypoints);

    /**
     * From 3D points
     *
     * @param keypoints detected 3D keypoints
     * @param scores confidence scores for each keypoint
     * @param confidence_threshold confidence threshold, filters out keypoints with confidence below threshold
     *  
     * @returns keypoints message
     */
    Keypoints& setKeypoints(const std::vector<Point3f>& points);
    Keypoints& setKeypoints(const std::vector<Point3f>& points, const std::vector<float>& scores);
    Keypoints& setKeypoints(const std::vector<Point3f>& points, const std::vector<float>& scores, float confidenceThreshold);

    /**
     * From 2D points
     *
     * @param keypoints detected 2D keypoints
     * @param scores confidence scores for each keypoint
     * @param confidence_threshold confidence threshold, filters out keypoints with confidence below threshold
     *  
     * @returns keypoints message
     */
    Keypoints& setKeypoints(const std::vector<Point2f>& points);
    Keypoints& setKeypoints(const std::vector<Point2f>& points, const std::vector<float>& scores);
    Keypoints& setKeypoints(const std::vector<Point2f>& points, const std::vector<float>& scores, float confidenceThreshold);

    DEPTHAI_SERIALIZE(Keypoints, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, keypoints);
};

}  // namespace dai
