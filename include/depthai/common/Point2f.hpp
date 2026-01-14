#pragma once

// std
#include <cstdint>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Point2f structure
 *
 * x and y coordinates that define a 2D point.
 */
struct Point2f {
    Point2f() = default;
    /**
     * Construct a 2D point with explicit coordinates.
     */
    Point2f(float x, float y) {
        this->x = x;
        this->y = y;
        this->hasNormalized = false;
    }
    /**
     * Construct a 2D point and explicitly mark normalization.
     */
    Point2f(float x, float y, bool normalized) {
        this->x = x;
        this->y = y;
        this->hasNormalized = true;
        this->normalized = normalized;
    }
    float x = 0, y = 0;
    bool normalized = false;
    bool hasNormalized = false;

    /**
     * Return whether the point is normalized to [0,1].
     */
    bool isNormalized() const {
        if(hasNormalized) {
            return normalized;
        }
        // When ambiguous, default to denormalized
        if((x == 0 || x == 1) && (y == 0 || y == 1)) return false;
        return x >= 0 && x <= 1 && y >= 0 && y <= 1;
    }
};

DEPTHAI_SERIALIZE_EXT(Point2f, x, y, normalized, hasNormalized);

}  // namespace dai
