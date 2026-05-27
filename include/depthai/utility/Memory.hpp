#pragma once

// std
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

// project
#include "depthai/utility/span.hpp"

namespace dai {

/*
class Memory {
    public:
    uint8_t* data = nullptr;
    size_t offset = 0;
    size_t size = 0;
    size_t maxSize = 0;
    linb::any holder;
    void* pool = nullptr;

    Memory(const Memory&) = delete;
    Memory& operator=(const Memory&) = delete;
    // std::function<void(Memory*)> accessStart, accessEnd;


    // Memory() = default;
    ~Memory() = default;

    Memory(uint8_t* data, size_t offset, size_t size, size_t maxSize) : data(data), offset(offset), size(size), maxSize(maxSize) {}
    Memory(uint8_t* data, size_t offset, size_t size, size_t maxSize, linb::any holder) : data(data), offset(offset), size(size), maxSize(maxSize),
holder(std::move(holder)) {}
    // Memory(linb::any holder, uint8_t* data, size_t size, size_t maxSize) : holder(holder), data(data), size(size), maxSize(maxSize) {}
    // Memory(linb::any holder, uint8_t* data, size_t size, size_t maxSize, void* pool) : holder(holder), data(data), size(size), maxSize(maxSize), pool(pool)
{} Memory(Memory&&) = default; Memory& operator=(Memory&&) = default;

    // public:
    static std::shared_ptr<Memory> create(uint8_t* data, size_t offset, size_t size, size_t maxSize) {
        return std::shared_ptr<Memory>(new Memory{data, offset, size, maxSize});
    }
    static std::shared_ptr<Memory> create(uint8_t* data, size_t offset, size_t size, size_t maxSize, std::function<void(Memory*)> del) {
        return std::shared_ptr<Memory>(new Memory{data, offset, size, maxSize}, del);
    }
    static std::shared_ptr<Memory> create(uint8_t* data, size_t offset, size_t size, size_t maxSize, linb::any holder) {
        return std::shared_ptr<Memory>(new Memory{data, offset, size, maxSize, std::move(holder)});
    }
    // std::shared_ptr<Memory> create(linb::any obj, uint8_t* data, size_t size, size_t maxSize) {
    //     return std::shared_ptr<Memory>(new Memory{obj, data, size, maxSize});
    // }
    // std::shared_ptr<Memory> create(linb::any obj, uint8_t* data, size_t size, size_t maxSize, std::function<void(Memory*)> del) {
    //     return std::shared_ptr<Memory>(new Memory{obj, data, size, maxSize}, del);
    // }
    // std::shared_ptr<Memory> create(linb::any obj, uint8_t* data, size_t size, size_t maxSize, void* pool, std::function<void(Memory*)> del = nullptr) {
    //     return std::shared_ptr<Memory>(new Memory{obj, data, size, maxSize, pool}, del);
    // }

    // template<typename T>
    // class Access : public span<T> {
    //     Memory& parent;
    //     public:
    //     Access(Memory& mem, span<T>::pointer ptr, span<T>::size_type count) : span(ptr, count), parent(mem) {
    //         parent.accessStart(&parent);
    //     }
    //     ~Access() {
    //         if(parent.accessEnd) {
    //             parent.accessEnd(&parent);
    //         }
    //     }
    // };
    // Access<uint8_t> getData() {
    //     return {*this, data, size};
    // }
    // Access<const uint8_t> getData() const {
    //     return {data, data + size};
    // }


    span<uint8_t> getData() {
        return {data + offset, size};
    }

    void setData(span<const uint8_t> d) {
        if(d.size() <= maxSize) {
            memcpy(data, d.data(), d.size());
            size = d.size();
        } else {
            throw std::invalid_argument("Memory object too small to set given data");
        }
    }
    void setSize(size_t size) {
        this->size = size;
    }
    size_t getSize() {
        return size;
    }

    size_t getMaxSize() {
        return maxSize;
    }
    void* getPool() {
        return pool;
    }
    const void* getPool() const {
        return pool;
    }
};
*/

// memory as interface
class Memory {
   public:
    virtual ~Memory();
    virtual span<std::uint8_t> getData() = 0;
    virtual span<const std::uint8_t> getData() const = 0;
    virtual std::size_t getMaxSize() const = 0;
    virtual std::size_t getOffset() const = 0;
    virtual void setSize(std::size_t) = 0;

    std::size_t getSize() const {
        return getData().size();
    };
};

class OffsetMemory : public Memory {
   public:
    virtual void setOffset(std::size_t) = 0;
    virtual span<std::uint8_t> getOffsetData() = 0;
    virtual span<const std::uint8_t> getOffsetData() const = 0;
    virtual std::shared_ptr<OffsetMemory> offset(std::size_t offset) = 0;
    std::size_t getOffsetSize() const {
        return getOffsetData().size();
    };
};

class ConvertedOffsetMemory : public OffsetMemory {
    std::shared_ptr<Memory> memory = nullptr;
    size_t dataOffset = 0;

   public:
    ConvertedOffsetMemory() = delete;
    explicit ConvertedOffsetMemory(std::shared_ptr<Memory> memory) : memory(std::move(memory)) {}

    span<std::uint8_t> getData() override {
        return memory->getData();
    }
    span<const std::uint8_t> getData() const override {
        return memory->getData();
    }
    std::size_t getMaxSize() const override {
        return memory->getMaxSize();
    }
    std::size_t getOffset() const override {
        return dataOffset;
    }
    void setSize(std::size_t size) override {
        memory->setSize(size);
        dataOffset = std::min(dataOffset, size);
    }
    void setOffset(std::size_t offset) override {
        dataOffset = std::min(dataOffset + offset, memory->getSize());
    }
    span<std::uint8_t> getOffsetData() override {
        const size_t size = memory->getSize();
        std::uint8_t* data = memory->getData().data();
        return {data + dataOffset, data + size};
    }
    span<const std::uint8_t> getOffsetData() const override {
        const size_t size = memory->getSize();
        const std::uint8_t* data = memory->getData().data();
        return {data + dataOffset, data + size};
    }
    std::shared_ptr<OffsetMemory> offset(std::size_t offset) override {
        auto mem = std::make_shared<ConvertedOffsetMemory>(memory);
        mem->dataOffset = dataOffset;
        mem->setOffset(offset);
        return mem;
    }

    std::shared_ptr<Memory> getInternal() {
        return memory;
    }

    static std::shared_ptr<OffsetMemory> convert(const std::shared_ptr<Memory>& memory) {
        if(memory == nullptr) return nullptr;
        auto dynOffsetMemory = std::dynamic_pointer_cast<OffsetMemory>(memory);
        if(dynOffsetMemory != nullptr) return dynOffsetMemory;
        return std::make_shared<ConvertedOffsetMemory>(memory);
    }

    template <typename T>
    static std::shared_ptr<T> cast(std::shared_ptr<OffsetMemory> memory) {
        if(memory == nullptr) return nullptr;
        auto omem = std::dynamic_pointer_cast<ConvertedOffsetMemory>(memory);
        if(omem != nullptr) {
            return std::dynamic_pointer_cast<T>(omem->getInternal());
        }
        return std::dynamic_pointer_cast<T>(memory);
    }
};

}  // namespace dai
