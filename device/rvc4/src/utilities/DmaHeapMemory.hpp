#pragma once

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "depthai/utility/OffsetMemory.hpp"

namespace dai {
namespace gate {

class DmaHeapMemory : public OffsetMemory {
   public:
    static constexpr const char* kDefaultHeapPath = "/dev/dma_heap/qcom,system";

    enum class CpuSyncAccess : std::uint64_t {
        Read = DMA_BUF_SYNC_READ,
        Write = DMA_BUF_SYNC_WRITE,
        ReadWrite = DMA_BUF_SYNC_RW,
    };

    class CpuAccessGuard {
       public:
        CpuAccessGuard(DmaHeapMemory& memory, CpuSyncAccess access)
            : memory(&memory), access(access) {
            this->memory->syncStartCpuAccess(access);
        }

        ~CpuAccessGuard() {
            if(memory != nullptr) {
                memory->syncEndCpuAccess(access);
            }
        }

        CpuAccessGuard(const CpuAccessGuard&) = delete;
        CpuAccessGuard& operator=(const CpuAccessGuard&) = delete;

        CpuAccessGuard(CpuAccessGuard&& other) noexcept : memory(other.memory), access(other.access) {
            other.memory = nullptr;
        }

        CpuAccessGuard& operator=(CpuAccessGuard&& other) noexcept {
            if(this != &other) {
                if(memory != nullptr) {
                    memory->syncEndCpuAccess(access);
                }
                memory = other.memory;
                access = other.access;
                other.memory = nullptr;
            }
            return *this;
        }

       private:
        DmaHeapMemory* memory;
        CpuSyncAccess access;
    };

    explicit DmaHeapMemory(std::size_t size, std::string heapPath = kDefaultHeapPath)
        : allocation(std::make_shared<Allocation>(size, std::move(heapPath))), logicalSize(size) {}

    ~DmaHeapMemory() override = default;

    DmaHeapMemory(const DmaHeapMemory&) = delete;
    DmaHeapMemory& operator=(const DmaHeapMemory&) = delete;

    span<std::uint8_t> getData() override {
        return {static_cast<std::uint8_t*>(allocation->mapping), logicalSize};
    }

    span<const std::uint8_t> getData() const override {
        return {static_cast<const std::uint8_t*>(allocation->mapping), logicalSize};
    }

    std::size_t getMaxSize() const override {
        return allocation->allocationSize;
    }

    std::size_t getOffset() const override {
        return dataOffset;
    }

    void setSize(std::size_t newSize) override {
        if(newSize > allocation->allocationSize) {
            throw std::invalid_argument("Cannot set dma heap memory size larger than allocation");
        }
        logicalSize = newSize;
        dataOffset = std::min(dataOffset, newSize);
    }

    void setOffset(std::size_t offset) override {
        dataOffset = std::min(dataOffset + offset, logicalSize);
    }

    span<std::uint8_t> getOffsetData() override {
        const size_t size = logicalSize;
        auto* data = static_cast<std::uint8_t*>(allocation->mapping);
        return {data + dataOffset, data + size};
    }

    span<const std::uint8_t> getOffsetData() const override {
        const size_t size = logicalSize;
        const auto* data = static_cast<const std::uint8_t*>(allocation->mapping);
        return {data + dataOffset, data + size};
    }

    std::shared_ptr<OffsetMemory> offset(std::size_t offset) override {
        auto mem = std::shared_ptr<DmaHeapMemory>(new DmaHeapMemory(allocation, logicalSize, dataOffset));
        mem->setOffset(offset);
        return mem;
    }

    int fd() const {
        return allocation->dmaBufFd;
    }

    /** Returns an ownership token shared by all views of this DMA allocation. */
    std::shared_ptr<void> getAllocationOwner() const {
        return allocation;
    }

    CpuAccessGuard scopedCpuReadAccess() {
        return CpuAccessGuard(*this, CpuSyncAccess::Read);
    }

    CpuAccessGuard scopedCpuWriteAccess() {
        return CpuAccessGuard(*this, CpuSyncAccess::Write);
    }

    CpuAccessGuard scopedCpuReadWriteAccess() {
        return CpuAccessGuard(*this, CpuSyncAccess::ReadWrite);
    }

    void syncStartCpuAccess(CpuSyncAccess access) {
        syncCpuAccess(DMA_BUF_SYNC_START | static_cast<std::uint64_t>(access));
    }

    void syncEndCpuAccess(CpuSyncAccess access) {
        syncCpuAccess(DMA_BUF_SYNC_END | static_cast<std::uint64_t>(access));
    }

   private:
    void syncCpuAccess(std::uint64_t flags) {
        dma_buf_sync sync = {};
        sync.flags = flags;

        if(ioctl(allocation->dmaBufFd, DMA_BUF_IOCTL_SYNC, &sync) < 0) {
            throw std::runtime_error("Failed to sync dma-buf CPU access: " + std::string(std::strerror(errno)));
        }
    }

    struct Allocation {
        explicit Allocation(std::size_t size, std::string heapPath) : heapPath(std::move(heapPath)), allocationSize(size) {
            if(allocationSize == 0) {
                throw std::invalid_argument("DmaHeapMemory size must be greater than 0");
            }

            const int heapFd = open(this->heapPath.c_str(), O_RDONLY | O_CLOEXEC);
            if(heapFd < 0) {
                throw std::runtime_error("Failed to open dma heap: " + this->heapPath);
            }

            dma_heap_allocation_data allocData = {};
            allocData.len = allocationSize;
            allocData.fd_flags = O_RDWR | O_CLOEXEC;
            allocData.heap_flags = 0;

            if(ioctl(heapFd, DMA_HEAP_IOCTL_ALLOC, &allocData) < 0) {
                close(heapFd);
                throw std::runtime_error("Failed to allocate buffer from dma heap: " + this->heapPath);
            }

            dmaBufFd = static_cast<int>(allocData.fd);
            close(heapFd);
            mapping = mmap(nullptr, allocationSize, PROT_READ | PROT_WRITE, MAP_SHARED, dmaBufFd, 0);
            if(mapping == MAP_FAILED) {
                close(dmaBufFd);
                dmaBufFd = -1;
                throw std::runtime_error("Failed to mmap dma heap allocation");
            }
        }

        ~Allocation() {
            if(mapping != MAP_FAILED) {
                munmap(mapping, allocationSize);
            }
            if(dmaBufFd >= 0) {
                close(dmaBufFd);
            }
        }

        Allocation(const Allocation&) = delete;
        Allocation& operator=(const Allocation&) = delete;

        std::string heapPath;
        int dmaBufFd{-1};
        void* mapping{MAP_FAILED};
        std::size_t allocationSize{0};
    };

    DmaHeapMemory(std::shared_ptr<Allocation> allocation, std::size_t logicalSize, std::size_t dataOffset)
        : allocation(std::move(allocation)), logicalSize(logicalSize), dataOffset(dataOffset) {}

    std::shared_ptr<Allocation> allocation;
    std::size_t logicalSize{0};

    std::size_t dataOffset = 0;
};

}  // namespace gate
}  // namespace dai
