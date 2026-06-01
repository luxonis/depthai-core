#pragma once

#include <memory>

#include "depthai/utility/Memory.hpp"

namespace dai {

/**
 * Memory interface that exposes a logical offset into an underlying byte buffer.
 */
class OffsetMemory : public Memory {
   public:
    /**
     * Advances the logical offset within the buffer.
     *
     * The resulting offset is clamped to the current buffer size.
     *
     * @param offset Number of bytes to advance from the current offset.
     */
    virtual void setOffset(std::size_t) = 0;

    /**
     * Gets a mutable view of the data starting at the current offset.
     *
     * @return Span over the remaining bytes after the offset.
     */
    virtual span<std::uint8_t> getOffsetData() = 0;

    /**
     * Gets a read-only view of the data starting at the current offset.
     *
     * @return Span over the remaining bytes after the offset.
     */
    virtual span<const std::uint8_t> getOffsetData() const = 0;

    /**
     * Creates a new offset-aware view advanced from the current offset.
     *
     * @param offset Number of bytes to advance from the current offset.
     * @return Shared pointer to a memory view with the updated offset applied.
     */
    virtual std::shared_ptr<OffsetMemory> offset(std::size_t offset) = 0;

    /**
     * Gets the number of bytes available from the current offset to the end of the buffer.
     *
     * @return Remaining buffer size after the offset.
     */
    std::size_t getOffsetSize() const {
        return getOffsetData().size();
    };
};

/**
 * Adapter that exposes any `Memory` instance through the `OffsetMemory` interface.
 */
class ConvertedOffsetMemory : public OffsetMemory {
    std::shared_ptr<Memory> memory = nullptr;
    size_t dataOffset = 0;

   public:
    ConvertedOffsetMemory() = delete;

    /**
     * Wraps an existing memory object with offset-aware access.
     *
     * @param memory Underlying memory object to expose through this adapter.
     */
    explicit ConvertedOffsetMemory(std::shared_ptr<Memory> memory) : memory(std::move(memory)) {
        if(this->memory == nullptr) throw std::invalid_argument("ConvertedOffsetMemory requires non-null memory");
    }

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

    /**
     * Gets the wrapped memory instance.
     *
     * @return Shared pointer to the underlying memory object.
     */
    std::shared_ptr<Memory> getInternal() {
        return memory;
    }

    /**
     * Converts a generic memory object into an `OffsetMemory`.
     *
     * Existing `OffsetMemory` instances are returned unchanged, while other memory
     * implementations are wrapped in `ConvertedOffsetMemory`.
     *
     * @param memory Memory object to convert.
     * @return Offset-aware view of the provided memory, or `nullptr` if `memory` is null.
     */
    static std::shared_ptr<OffsetMemory> convert(const std::shared_ptr<Memory>& memory) {
        if(memory == nullptr) return nullptr;
        auto dynOffsetMemory = std::dynamic_pointer_cast<OffsetMemory>(memory);
        if(dynOffsetMemory != nullptr) return dynOffsetMemory;
        return std::make_shared<ConvertedOffsetMemory>(memory);
    }

    /**
     * Casts an offset-aware memory object back to a concrete memory type.
     *
     * If the offset-aware instance is a `ConvertedOffsetMemory`, the cast is performed
     * against its wrapped memory object.
     *
     * @tparam T Concrete memory type to cast to.
     * @param memory Offset-aware memory object to cast.
     * @return Shared pointer to `T`, or `nullptr` if the cast fails.
     */
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
