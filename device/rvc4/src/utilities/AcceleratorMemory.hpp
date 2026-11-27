#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "depthai/utility/Memory.hpp"
#include "depthai/utility/OffsetMemory.hpp"
#include "vendor/dai_device_common.h"

namespace dai {

class AcceleratorMemory final : public OffsetMemory, public std::enable_shared_from_this<AcceleratorMemory> {
    struct Private {};

   public:
    AcceleratorMemory(Private, dai_device_mem* mem, std::size_t size, std::shared_ptr<const void> keepAlive);
    ~AcceleratorMemory() override;
    AcceleratorMemory(const AcceleratorMemory&) = delete;
    AcceleratorMemory& operator=(const AcceleratorMemory&) = delete;

    std::uint8_t* data();
    const std::uint8_t* data() const;

    std::size_t size() const;

    int fd() const;

    dai_device_mem* handle() const noexcept {
        return mem;
    }

    static std::shared_ptr<AcceleratorMemory> create(std::size_t size);

    static std::shared_ptr<AcceleratorMemory> convertZeroCopy(std::shared_ptr<Memory> data);  // nullptr when data is not DMA-BUF backed

    static std::shared_ptr<AcceleratorMemory> convert(std::shared_ptr<Memory> data, std::size_t size = 0);  // size: copy buffer size, 0 = the source size

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
    std::shared_ptr<const void> keepAlive;  // Must stay valid while registered: the zero-copy source, or the buffer of an offset() view
    std::uint8_t* base = nullptr;
    std::size_t total = 0;
    std::size_t logicalSize = 0;
    std::size_t currentOffset = 0;
    int bufferFd = -1;
};

}  // namespace dai
