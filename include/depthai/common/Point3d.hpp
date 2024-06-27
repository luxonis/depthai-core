#pragma once

// std
#include <cstdint>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Point3d structure
 *
 * x,y,z coordinates that define a 3D point.
 */
struct Point3d {
    Point3d() = default;
    Point3d(double x, double y, double z) : x(x), y(y), z(z) {}
    double x = 0, y = 0, z = 0;
};

DEPTHAI_SERIALIZE_EXT(Point3d, x, y, z);

}  // namespace dai
