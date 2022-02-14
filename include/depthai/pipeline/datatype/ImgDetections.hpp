#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgDetections& dets;

   public:
    /// Construct ImgDetections message
    ImgDetections();
    explicit ImgDetections(std::shared_ptr<RawImgDetections> ptr);
    virtual ~ImgDetections() = default;

    /// Detections
    std::vector<ImgDetection>& detections;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    ImgDetections& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    ImgDetections& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    ImgDetections& setSequenceNum(int64_t sequenceNum);

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
