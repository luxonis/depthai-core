#pragma once

// std
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>

// project
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/SharedMemory.hpp"
#include "gst/allocators/gstfdmemory.h"

// libraries
#include <gstreamer-1.0/gst/allocators/gstdmabuf.h>
#include <gstreamer-1.0/gst/gst.h>

namespace dai {
namespace gate {

// memory as interface
class GstBufferMemory : public Memory {
   public:
    GstBuffer* buf = nullptr;
    GstMapInfo map;

    GstBufferMemory() = default;
    GstBufferMemory& operator=(const GstBufferMemory&) = delete;
    GstBufferMemory(const GstBufferMemory&) = delete;
    GstBufferMemory(GstBuffer* buffer, bool writable = false) {
        // add reference
        buf = gst_buffer_ref(buffer);
        if(buf == nullptr) {
            throw std::runtime_error("Couldn't add reference to buffer");
        }

        // map
        if(writable) {
            if(!gst_buffer_map(buf, &map, GST_MAP_READWRITE)) {
                throw std::runtime_error("Couldn't map buffer's memory for writing");
            }
        } else {
            if(!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                throw std::runtime_error("Couldn't map buffer's memory");
            }
        }

    }
    ~GstBufferMemory() {
        if(buf) {
            gst_buffer_unmap(buf, &map);
            gst_buffer_unref(buf);
        }
    }

    span<std::uint8_t> getData() override {
        return {map.data, map.size};
    }
    span<const std::uint8_t> getData() const override {
        return {map.data, map.size};
    }
    std::size_t getMaxSize() const {
        return map.maxsize;
    }
    std::size_t getOffset() const {
        return 0;
    }
    void setSize(std::size_t size) override {
        map.size = size;
    }
};

}  // namespace gate
}  // namespace dai
