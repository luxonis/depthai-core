#pragma once

// std
#include <cstddef>
#include <cstdint>
#include <memory>

// project
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/OffsetMemory.hpp"
#include "vendor/dai_device_common.h"

namespace dai {

/**
 * An accelerator buffer of the vendor module (dai_device_mem of vendor/dai_device_common.h): a buffer that the
 * vendor module allocates, or the DMA-BUF of other memory that it registers without a copy. The accelerator units
 * and the DSP offload take their buffers as AcceleratorMemory.
 */
class AcceleratorMemory final : public OffsetMemory, public std::enable_shared_from_this<AcceleratorMemory> {
    struct Private {};

   public:
    AcceleratorMemory(Private, dai_device_mem* mem, std::size_t size, std::shared_ptr<const void> keepAlive);
    ~AcceleratorMemory() override;
    AcceleratorMemory(const AcceleratorMemory&) = delete;
    AcceleratorMemory& operator=(const AcceleratorMemory&) = delete;

    /// Pointer to the first byte after the current offset.
    std::uint8_t* data();
    const std::uint8_t* data() const;

    /// Number of bytes from the current offset to the end of the usable buffer.
    std::size_t size() const;

    /// DMA-BUF file descriptor of the underlying allocation.
    int fd() const;

    /// The handle of the vendor module, for the C interface
    dai_device_mem* handle() const noexcept {
        return mem;
    }

    /// Allocates a new accelerator buffer of `size` bytes.
    static std::shared_ptr<AcceleratorMemory> create(std::size_t size);

    /**
     * Wraps DMA-BUF backed memory (camera HAL, DMA heap or GStreamer fd memory, or an existing AcceleratorMemory)
     * without copying. Returns nullptr for other memory.
     */
    static std::shared_ptr<AcceleratorMemory> convertZeroCopy(std::shared_ptr<Memory> data);

    /**
     * convertZeroCopy, or a copy into a new buffer when zero-copy is not possible.
     * `size` is the size of the new buffer in the copy case (0: the source size).
     */
    static std::shared_ptr<AcceleratorMemory> convert(std::shared_ptr<Memory> data, std::size_t size = 0);

    // Memory and OffsetMemory
    span<std::uint8_t> getData() override;
    span<const std::uint8_t> getData() const override;
    std::size_t getMaxSize() const override;
    std::size_t getOffset() const override;
    void setSize(std::size_t newSize) override;
    void setOffset(std::size_t offset) override;
    span<std::uint8_t> getOffsetData() override;
    span<const std::uint8_t> getOffsetData() const override;
    std::shared_ptr<OffsetMemory> offset(std::size_t offset) override;

   private:
    dai_device_mem* mem = nullptr;
    // The memory that must stay valid while the buffer is registered: the source of a zero-copy buffer, or the
    // buffer of a view made by offset()
    std::shared_ptr<const void> keepAlive;
    std::uint8_t* base = nullptr;
    std::size_t total = 0;
    std::size_t logicalSize = 0;
    std::size_t currentOffset = 0;
    int bufferFd = -1;
};

}  // namespace dai
