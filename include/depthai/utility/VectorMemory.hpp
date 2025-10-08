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

    ~VectorMemory() override;
    span<std::uint8_t> getData() override;
    span<const std::uint8_t> getData() const override;
    std::size_t getMaxSize() const override;
    std::size_t getOffset() const override;
    void setSize(std::size_t size) override;
};

}  // namespace dai
