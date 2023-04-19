#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/utility/span.hpp"

namespace dai {

/// Base message - buffer of binary data
class Buffer : public ADatatype {
    Serialized serialize() const override;

   public:
    /// Creates Buffer message
    Buffer();
    Buffer(size_t size);
    explicit Buffer(std::shared_ptr<dai::RawBuffer> ptr);
    virtual ~Buffer() = default;

    // helpers
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

    /**
     * @param data Moves data to internal buffer
     */
    void setData(std::vector<std::uint8_t>&& data);
};

}  // namespace dai
