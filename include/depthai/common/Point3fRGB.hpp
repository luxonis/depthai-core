#pragma once

// std
#include <cstdint>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Point3fRGB structure
 *
 * x,y,z coordinates and RGB color values that define a 3D point with color.
 */
struct Point3fRGB {
    Point3fRGB() = default;
    Point3fRGB(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) : x(x), y(y), z(z), r(r), g(g), b(b) {}
    float x = 0, y = 0, z = 0;
    uint8_t r = 0, g = 0, b = 0;
};

DEPTHAI_SERIALIZE_EXT(Point3fRGB, x, y, z, r, g, b);

}  // namespace dai