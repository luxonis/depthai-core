#pragma once

// std
#include <cstddef>
#include <cstdint>
#include <memory>

// project
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/OffsetMemory.hpp"

namespace dai {

class EvaDataMemory : public OffsetMemory {
   public:
    ~EvaDataMemory() override = default;

    /// Pointer to the first byte after the current offset.
    virtual std::uint8_t* data() = 0;
    virtual const std::uint8_t* data() const = 0;

    /// Number of bytes from the current offset to the end of the usable buffer.
    virtual std::size_t size() const = 0;

    /// DMA-BUF file descriptor of the underlying allocation.
    virtual int fd() const = 0;

    /// Allocates a new accelerator buffer of `size` bytes.
    static std::shared_ptr<EvaDataMemory> create(std::size_t size);

    /**
     * Wraps DMA-BUF backed memory (memory that implements dai::gate::DmaBufMemory with a valid
     * file descriptor, or an existing EvaDataMemory) without copying. Returns nullptr for other memory.
     */
    static std::shared_ptr<EvaDataMemory> convertZeroCopy(std::shared_ptr<Memory> data);

    /**
     * convertZeroCopy, or a copy into a new buffer when zero-copy is not possible.
     * `size` is the size of the new buffer in the copy case (0: the source size).
     */
    static std::shared_ptr<EvaDataMemory> convert(std::shared_ptr<Memory> data, std::size_t size = 0);
};

}  // namespace dai
