#pragma once

#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Point2f structure
 *
 * x and y coordinates that define a 2D point.
 */
struct Point2f {
    Point2f() = default;
    Point2f(float x, float y) {
        this->x = x;
        this->y = y;
        this->hasNormalized = false;
    }
    Point2f(float x, float y, bool normalized) {
        this->x = x;
        this->y = y;
        this->hasNormalized = true;
        this->normalized = normalized;
    }
    float x = 0, y = 0;
    bool normalized = false;
    bool hasNormalized = false;

    bool isNormalized() const {
        if(hasNormalized) {
            return normalized;
        }
        if((x == 0 || x == 1) && (y == 0 || y == 1)) return false;  // Ambiguous 0/1 coordinates count as denormalized
        return x >= 0 && x <= 1 && y >= 0 && y <= 1;
    }
};

DEPTHAI_SERIALIZE_EXT_DECL(Point2f, x, y, normalized, hasNormalized);

}  // namespace dai
