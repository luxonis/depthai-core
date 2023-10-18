#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawOccupancyPool.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * OccupancyPool message.
 */
class OccupancyPool : public Buffer {
    Serialized serialize() const override;
    RawOccupancyPool& rawdata;

   public:
    /**
     * Construct OccupancyPool message.
     */
    OccupancyPool();
    explicit OccupancyPool(std::shared_ptr<RawOccupancyPool> ptr);
    virtual ~OccupancyPool() = default;

    std::vector<std::vector<int>>& occupancyPool;
    std::vector<std::vector<std::vector<int>>>& occupancyPool3d;

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
    OccupancyPool& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    OccupancyPool& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    OccupancyPool& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
