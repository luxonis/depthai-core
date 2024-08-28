#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/span.hpp"

namespace dai {

/// Base message - buffer of binary data
class Buffer : public ADatatype {
   public:
    Buffer() = default;
    Buffer(size_t size);
    Buffer(const char *name);
    Buffer(const char *name, size_t size);
    Buffer(long fd);
    Buffer(long fd, size_t size);
    virtual ~Buffer() = default;

    virtual void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Buffer;
    };

    /**
     * @brief Get non-owning reference to internal buffer
     * @returns Reference to internal buffer
     */
    span<uint8_t> getData();
    span<const uint8_t> getData() const;

    /**
     * @param data Copies data to internal buffer
     */
    void setData(const std::vector<std::uint8_t>& data);
    void setData(const long fd);

    /**
     * @param data Moves data to internal buffer
     */
    void setData(std::vector<std::uint8_t>&& data);

    /**
     * Retrieves timestamp related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;

    /**
     * Retrieves timestamp directly captured from device's monotonic clock,
     * not synchronized to host time. Used mostly for debugging
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice() const;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    void setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    void setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    int64_t getSequenceNum() const;

    /**
     * Sets image sequence number
     */
    void setSequenceNum(int64_t sequenceNum);

    virtual span<const uint8_t> getRecordData() const;

    // TODO(Morato) // Make this private
    int64_t sequenceNum = 0;  // increments for each message
    Timestamp ts = {};        // generation timestamp, synced to host time
    Timestamp tsDevice = {};  // generation timestamp, direct device monotonic clock
    DEPTHAI_SERIALIZE(Buffer, sequenceNum, ts, tsDevice);
};

}  // namespace dai
