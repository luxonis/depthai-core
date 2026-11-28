#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "depthai/utility/span.hpp"

namespace dai {

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

}  // namespace dai
