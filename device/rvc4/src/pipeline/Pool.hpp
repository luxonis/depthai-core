#pragma once

#include <gst/memory/gstmempool.h>

#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "utilities/AcceleratorMemory.hpp"
#include "utilities/GstBufferMemory.hpp"
namespace dai {
namespace gate {

template <typename T, typename = typename std::enable_if<std::is_base_of<dai::ADatatype, T>::value>::type>
class Pool : public std::enable_shared_from_this<Pool<T>> {
   public:
    Pool(size_t numMessages, size_t maxSize, bool caching = true, bool invalidate = true) : pool(numMessages) {
        (void)invalidate;
        (void)caching;
        if(maxSize == 0) {
            throw std::runtime_error("Max size must be greater than 0");
        }
        if(numMessages == 0) {
            throw std::runtime_error("Number of messages must be greater than 0");
        }

        if(enableIon) {

            GstStructure* config = NULL;
            GstAllocator* fdAllocator = NULL;

            fdMemoryPool = gst_mem_buffer_pool_new(GST_MEMORY_BUFFER_POOL_TYPE_ION);
            if(!fdMemoryPool) {
                throw std::runtime_error("Failed to create memory pool!");
            }
            config = gst_buffer_pool_get_config(fdMemoryPool);
            if(!config) {
                g_object_unref(fdMemoryPool);
                throw std::runtime_error("Failed to get memory pool configuration!");
            }
            gst_buffer_pool_config_set_params(config, NULL, maxSize, numMessages, numMessages);
            fdAllocator = gst_fd_allocator_new();
            if(!fdAllocator) {
                g_object_unref(fdMemoryPool);
                throw std::runtime_error("Failed to create memory pool allocator!");
            }
            gst_buffer_pool_config_set_allocator(config, fdAllocator, NULL);

            if(!gst_buffer_pool_set_config(fdMemoryPool, config)) {
                g_object_unref(fdMemoryPool);
                throw std::runtime_error("Failed to set memory pool configuration!");
            }

            g_object_unref(fdAllocator);

            if(!gst_buffer_pool_is_active(fdMemoryPool) && !gst_buffer_pool_set_active(fdMemoryPool, TRUE)) {
                throw std::runtime_error("Failed to activate memory pool!");
            }
        }

        for(size_t i = 0; i < numMessages; ++i) {
            std::unique_ptr<T> msg = std::make_unique<T>();
            if(enableIon) {
                auto gstMemory = std::make_shared<GstBufferMemory>();
                gst_buffer_pool_acquire_buffer(fdMemoryPool, &gstMemory->buf, NULL);
                if(!gst_is_fd_memory(gst_buffer_peek_memory(gstMemory->buf, 0))) {
                    throw std::runtime_error("Failed to acquire memory from memory pool!");
                }
                if(gst_buffer_map(gstMemory->buf, &gstMemory->map, GST_MAP_READWRITE)) {
                } else {
                    throw std::runtime_error("Failed to map memory!");
                }
                msg->data = gstMemory;
            } else {
                msg->data = AcceleratorMemory::create(maxSize);
            }
            pool.push(std::move(msg));
        }
    }

    ~Pool() {
        if(fdMemoryPool) {
            if(gst_buffer_pool_is_active(fdMemoryPool)) gst_buffer_pool_set_active(fdMemoryPool, FALSE);
            gst_object_unref(fdMemoryPool);
            fdMemoryPool = nullptr;
        }
    }

    std::shared_ptr<T> get() {
        std::unique_ptr<T> msg;
        if(!pool.waitAndPop(msg)) {
            throw std::runtime_error("Failed to retrieve object from pool");
        }

        // Capture a weak_ptr to *this
        std::weak_ptr<Pool<T>> weak_this = this->shared_from_this();

        return std::shared_ptr<T>(msg.release(), [weak_this](T* msg) {
            std::unique_ptr<T> newMsg = std::make_unique<T>();
            newMsg->data = std::move(msg->data);
            delete msg;

            // Check if the Pool still exists
            if(auto shared_this = weak_this.lock()) {
                shared_this->pool.push(std::move(newMsg));
            }
            // else: Pool has been destroyed, newMsg will be destroyed when it goes out of scope
        });
    }

   private:
    dai::LockingQueue<std::unique_ptr<T>> pool;

    private:
    GstBufferPool* fdMemoryPool = nullptr;
    bool enableIon = false;
};

}  // namespace gate
}  // namespace dai