#pragma once

// std
#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Size2f structure
 *
 * width, height values define the size of the shape/frame
 */
struct Size2f {
    Size2f() = default;
    Size2f(float width, float height) : width(width), height(height), hasNormalized(false) {}
    Size2f(float width, float height, bool normalized) : width(width), height(height), normalized(normalized), hasNormalized(true) {}
    float width = 0, height = 0;
    bool normalized = false;
    bool hasNormalized = false;

    bool isNormalized() const {
        if(hasNormalized) return normalized;
        // When ambiguous, default to denormalized
        if((width == 0 || width == 1) && (height == 0 || height == 1)) return false;
        return width >= 0 && width <= 1 && height >= 0 && height <= 1;
    }
};

DEPTHAI_SERIALIZE_EXT(Size2f, width, height, normalized, hasNormalized);

}  // namespace dai
