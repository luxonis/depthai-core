#pragma once

// std
#include <cstdint>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Color structure
 *
 * r,g,b,a color values with values in range [0.0, 1.0]
 */
struct Color {
    Color() = default;
    /**
     * @param r Red value
     * @param g Green value
     * @param b Blue value
     * @param a Alpha value
     * @throws std::invalid_argument if r,g,b,a values are not in range [0.0, 1.0]
     */
    Color(float r, float g, float b, float a) {
        // r,g,b,a values should be in range [0.0, 1.0]
        auto check = [](float val) -> float {
            const float epsilon = 1e-6f;
            if(val < (0.0f - epsilon) || val > (1.0f + epsilon)) {
                throw std::invalid_argument("Color values should be in range [0.0, 1.0]");
            }
            return val;
        };
        this->r = check(r);
        this->g = check(g);
        this->b = check(b);
        this->a = check(a);
    }
    float r = 0.0f, g = 0.0f, b = 0.0f, a = 0.0f;
};

DEPTHAI_SERIALIZE_EXT(Color, r, g, b, a);

}  // namespace dai
