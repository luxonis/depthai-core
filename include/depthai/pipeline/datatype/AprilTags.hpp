#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTags message.
 */
class AprilTags : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAprilTags& rawdata;

   public:
    /**
     * Construct AprilTags message.
     */
    AprilTags();
    explicit AprilTags(std::shared_ptr<RawAprilTags> ptr);
    virtual ~AprilTags() = default;

    std::vector<AprilTag>& aprilTags;

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

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    AprilTags& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    AprilTags& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    AprilTags& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
