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
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;
    using Buffer::getSequenceNum;

    /**
     * Construct VisionHealthMetrics message.
     */
    VisionHealthMetrics();
    explicit VisionHealthMetrics(std::shared_ptr<RawVisionHealthMetrics> ptr);
    virtual ~VisionHealthMetrics() = default;

    std::unordered_map<VisionHealthMetricTypes, VisionHealthMetric>& visionHealthMetrics;

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
