#pragma once

// std
#include <cstdint>

// project
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Quaterniond structure
 *
 * qx,qy,qz,qw coordinates that define a 3D point orientation.
 */
struct Quaterniond {
    Quaterniond() = default;
    Quaterniond(double qx, double qy, double qz, double qw) : qx(qx), qy(qy), qz(qz), qw(qw) {}
    double qx = 0, qy = 0, qz = 0, qw = 1;
};

DEPTHAI_SERIALIZE_EXT(Quaterniond, qx, qy, qz, qw);

}  // namespace dai
