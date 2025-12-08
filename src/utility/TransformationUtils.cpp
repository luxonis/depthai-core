#include "depthai/utility/TransformationUtils.hpp"

namespace dai {

void transformPointCloudRGBA(
    std::vector<Point3fRGBA>& points,
    const Mat3 R,
    const Vec3 t
) {
    for(auto& p : points) {
        float x = p.x;
        float y = p.y;
        float z = p.z;

        float X =
            R[0][0] * x + R[0][1] * y + R[0][2] * z + t[0];
        float Y =
            R[1][0] * x + R[1][1] * y + R[1][2] * z + t[1];
        float Z =
            R[2][0] * x + R[2][1] * y + R[2][2] * z + t[2];

        p.x = X;
        p.y = Y;
        p.z = Z;
    }
}

} // namespace dai