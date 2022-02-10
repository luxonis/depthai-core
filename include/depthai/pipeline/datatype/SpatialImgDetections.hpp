#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * SpatialImgDetections message. Carries detection results together with spatial location data
 */
class SpatialImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialImgDetections& dets;

   public:
    /**
     * Construct SpatialImgDetections message.
     */
    SpatialImgDetections();
    explicit SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr);
    virtual ~SpatialImgDetections() = default;

    /**
     * Detection results.
     */
    std::vector<SpatialImgDetection>& detections;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    SpatialImgDetections& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    SpatialImgDetections& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    SpatialImgDetections& setSequenceNum(int64_t sequenceNum);

    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;

    /**
     * Retrieves image timestamp directly captured from device's monotonic clock,
     * not synchronized to host time. Used mostly for debugging
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice() const;

    /**
     * Retrieves image sequence number
     */
    int64_t getSequenceNum() const;
};

}  // namespace dai
