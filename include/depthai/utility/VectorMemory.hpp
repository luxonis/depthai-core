#pragma once

// std
#include <cstdint>
#include <cstring>
#include <functional>

// project
#include "depthai/utility/Memory.hpp"

namespace dai {

// memory as interface
class VectorMemory : public std::vector<std::uint8_t>, public Memory {
   public:
    // using std::vector<std::uint8_t>::vector;
    VectorMemory() = default;
    VectorMemory(const std::vector<std::uint8_t>& d) : vector(std::move(d)) {}
    VectorMemory(std::vector<std::uint8_t>&& d) : vector(std::move(d)) {}
    VectorMemory& operator=(std::vector<std::uint8_t>&& d) {
        std::vector<std::uint8_t>::operator=(std::move(d));
        return *this;
    }
    span<std::uint8_t> getData() override {
        return {data(), size()};
    }
    span<const std::uint8_t> getData() const override {
        return {data(), size()};
    }
    std::size_t getMaxSize() const override {
        return capacity();
    }
    std::size_t getOffset() const override {
        return 0;
    }
    void setSize(std::size_t size) override {
        resize(size);
    }

#if defined(__clang__)
    ~VectorMemory() override;
#else
    virtual ~VectorMemory() = default;
#endif
};

}  // namespace dai
