
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
    Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}
    float r = 0.0, g = 0.0, b = 0.0, a = 0.0;
};

DEPTHAI_SERIALIZE_EXT(Color, r, g, b, a);

}  // namespace dai
