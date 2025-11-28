#pragma once
#include <vector>
#include "depthai/pipeline/datatype/PointCloudData.hpp"

// Simple 3x3 matrix + 3x1 translation vector
using Mat3 = float[3][3];
using Vec3 = float[3];

namespace dai {

void transformPointCloudRGBA(
    std::vector<Point3fRGBA>& points,
    const Mat3 R,
    const Vec3 t
);

} // namespace dai