#pragma once

#include <hardware/camera3.h>
#include <sys/mman.h>

// std
#include <cstdint>
#include <cstring>
#include <memory>

// project
#include "camera/CameraPool.hpp"
#include "depthai/utility/Memory.hpp"
#include "utilities/ErrorMacros.hpp"

namespace dai {
namespace gate {

// memory as interface
class HalBufferMemory : public Memory {
   public:
    uint64_t size{0};
    uint8_t* data{nullptr};
    camera3_stream_buffer_t buf{};
    const CameraPool::private_handle_t* privHnd{nullptr};
    std::shared_ptr<CameraPool> pool;

    HalBufferMemory(const camera3_stream_buffer_t* buffer, std::shared_ptr<CameraPool> pool) {
        buf = *buffer;
        this->pool = pool;
        privHnd = static_cast<const CameraPool::private_handle_t*>(*buffer->buffer);
        uint8_t* mappedBuffer = (uint8_t*)mmap(nullptr, privHnd->size, PROT_READ | PROT_WRITE, MAP_SHARED, privHnd->fd, 0);
        if(mappedBuffer == NULL) {
            fprintf(stderr, "Unable to map HalBufferMemory fd -> memory\n");
            return;
        }
        data = mappedBuffer;
        size = privHnd->size;
        DAI_TRACE("Reserved buffer %p", static_cast<void*>(getData().data()));
    }
    ~HalBufferMemory() {
        munmap(data, privHnd->size);
        if(pool) {
            pool->returnBuffer(&buf);
        } else {
            DAI_ERROR("pool was null");
        }
        DAI_TRACE("Released buffer %p", static_cast<void*>(getData().data()));
    }

    span<std::uint8_t> getData() override {
        return {data, size};
    }
    span<const std::uint8_t> getData() const override {
        return {data, size};
    }
    std::size_t getSize() const {
        return size;
    }
    std::size_t getMaxSize() const override {
        return size;
    }
    std::size_t getOffset() const override {
        return 0;
    }
    void setSize(std::size_t newSize) override {
        fprintf(stderr, "Set size not implemented for HalBufferMemory, ignoring call!!!\n");
    }
};

}  // namespace gate
}  // namespace dai

#undef DEBUG
