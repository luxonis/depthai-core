#pragma once

#include <memory>

#include "depthai/utility/Memory.hpp"

namespace dai {

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
        dataOffset = std::min(dataOffset, memory->getSize());
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

}
