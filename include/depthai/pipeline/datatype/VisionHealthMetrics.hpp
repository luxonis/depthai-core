#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawVisionHealthMetrics.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * VisionHealthMetrics message.
 */
class VisionHealthMetrics : public Buffer {
    Serialized serialize() const override;
    RawVisionHealthMetrics& rawdata;

   public:
    /**
     * Construct VisionHealthMetrics message.
     */
    VisionHealthMetrics();
    explicit VisionHealthMetrics(std::shared_ptr<RawVisionHealthMetrics> ptr);
    virtual ~VisionHealthMetrics() = default;

    std::unordered_map<VisionHealthMetricTypes, VisionHealthMetric>& visionHealthMetrics;

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
    VisionHealthMetrics& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    VisionHealthMetrics& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    VisionHealthMetrics& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
