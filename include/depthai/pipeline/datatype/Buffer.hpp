#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>
#include <variant>

#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/span.hpp"

namespace dai {
class ImgAnnotations;
class ImgFrame;
using VisualizeType = std::variant<std::shared_ptr<ImgAnnotations>, std::shared_ptr<ImgFrame>, std::monostate>;

/// Base message - buffer of binary data
class Buffer : public ADatatype {
   public:
    Buffer() = default;
    Buffer(size_t size);
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

    /**
     * Get visualizable message
     @return Visualizable message, either ImgFrame, ImgAnnotations or std::monostate (None)
    */
    virtual dai::VisualizeType getVisualizationMessage() const;

    // TODO(Morato) // Make this private
    int64_t sequenceNum = 0;  // increments for each message
    Timestamp ts = {};        // generation timestamp, synced to host time
    Timestamp tsDevice = {};  // generation timestamp, direct device monotonic clock
    DEPTHAI_SERIALIZE(Buffer, sequenceNum, ts, tsDevice);
};

}  // namespace dai
