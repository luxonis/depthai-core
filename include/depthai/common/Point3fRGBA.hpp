#pragma once

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Point3fRGBA structure
 *
 * x,y,z coordinates and RGB color values that define a 3D point with color and alpha.
 */
struct Point3fRGBA {
    Point3fRGBA() = default;
    Point3fRGBA(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) : x(x), y(y), z(z), r(r), g(g), b(b), a(a) {}
    float x = 0, y = 0, z = 0;
    uint8_t r = 0, g = 0, b = 0, a = 255;
};

DEPTHAI_SERIALIZE_EXT(Point3fRGBA, x, y, z, r, g, b, a);

}  // namespace dai
