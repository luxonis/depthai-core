#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawBuffer structure
struct RawBuffer {
    virtual ~RawBuffer() = default;

    virtual void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
        (void)metadata;
        datatype = DatatypeEnum::Buffer;
    };
};

}  // namespace dai
