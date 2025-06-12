#include "depthai/pipeline/datatype/TransformData.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace dai {

TransformData::TransformData() {}
TransformData::TransformData(const Transform& transform, const std::string& frameID, const std::string& parentFrameID)
    : transform(transform), frameID(frameID), parentFrameID(parentFrameID) {}
TransformData::TransformData(const std::array<std::array<double, 4>, 4>& data, const std::string& frameID, const std::string& parentFrameID)
    : transform({data}), frameID(frameID), parentFrameID(parentFrameID) {}
TransformData::TransformData(
    double x, double y, double z, double qx, double qy, double qz, double qw, const std::string& frameID, const std::string& parentFrameID)
    : frameID(frameID), parentFrameID(parentFrameID) {
    // x,y,z,qx,qy,qz,qw to homography matrix
    double n = 1.0 / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    qx *= n;
    qy *= n;
    qz *= n;
    qw *= n;
    transform.matrix = {{{1.0 - 2.0 * qy * qy - 2.0 * qz * qz, 2.0 * qx * qy - 2.0 * qz * qw, 2.0 * qx * qz + 2.0 * qy * qw, x},
                         {2.0 * qx * qy + 2.0 * qz * qw, 1.0 - 2.0 * qx * qx - 2.0 * qz * qz, 2.0 * qy * qz - 2.0 * qx * qw, y},
                         {2.0 * qx * qz - 2.0 * qy * qw, 2.0 * qy * qz + 2.0 * qx * qw, 1.0 - 2.0 * qx * qx - 2.0 * qy * qy, z},
                         {0.0, 0.0, 0.0, 1.0}}};
}
TransformData::TransformData(double x, double y, double z, double roll, double pitch, double yaw, const std::string& frameID, const std::string& parentFrameID)
    : frameID(frameID), parentFrameID(parentFrameID) {
    // x,y,z,r,p,yw to homography matrix
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    transform.matrix = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x},
                         {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y},
                         {-sp, cp * sr, cp * cr, z},
                         {0.0, 0.0, 0.0, 1.0}}};
}

Point3d TransformData::getTranslation() const {
    double x = transform.matrix[0][3];
    double y = transform.matrix[1][3];
    double z = transform.matrix[2][3];
    return Point3d(x, y, z);
}
Point3d TransformData::getRotationEuler() const {
    double r, p, yw;
    p = -asin(transform.matrix[2][0]);
    if(fabs(p) < M_PI / 2) {
        r = atan2(transform.matrix[2][1], transform.matrix[2][2]);
        yw = atan2(transform.matrix[1][0], transform.matrix[0][0]);
    } else {
        r = atan2(-transform.matrix[0][1], transform.matrix[1][1]);
        yw = 0;
    }
    return Point3d(r, p, yw);
}
Quaterniond TransformData::getQuaternion() const {
    double qx, qy, qz, qw;
    double tr = transform.matrix[0][0] + transform.matrix[1][1] + transform.matrix[2][2];
    if(tr > 0) {
        double S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (transform.matrix[2][1] - transform.matrix[1][2]) / S;
        qy = (transform.matrix[0][2] - transform.matrix[2][0]) / S;
        qz = (transform.matrix[1][0] - transform.matrix[0][1]) / S;
    } else if(transform.matrix[0][0] > transform.matrix[1][1] && transform.matrix[0][0] > transform.matrix[2][2]) {
        double S = sqrt(1.0 + transform.matrix[0][0] - transform.matrix[1][1] - transform.matrix[2][2]) * 2;
        qw = (transform.matrix[2][1] - transform.matrix[1][2]) / S;
        qx = 0.25 * S;
        qy = (transform.matrix[0][1] + transform.matrix[1][0]) / S;
        qz = (transform.matrix[0][2] + transform.matrix[2][0]) / S;
    } else if(transform.matrix[1][1] > transform.matrix[2][2]) {
        double S = sqrt(1.0 + transform.matrix[1][1] - transform.matrix[0][0] - transform.matrix[2][2]) * 2;
        qw = (transform.matrix[0][2] - transform.matrix[2][0]) / S;
        qx = (transform.matrix[0][1] + transform.matrix[1][0]) / S;
        qy = 0.25 * S;
        qz = (transform.matrix[1][2] + transform.matrix[2][1]) / S;
    } else {
        double S = sqrt(1.0 + transform.matrix[2][2] - transform.matrix[0][0] - transform.matrix[1][1]) * 2;
        qw = (transform.matrix[1][0] - transform.matrix[0][1]) / S;
        qx = (transform.matrix[0][2] + transform.matrix[2][0]) / S;
        qy = (transform.matrix[1][2] + transform.matrix[2][1]) / S;
        qz = 0.25 * S;
    }
    return Quaterniond(qx, qy, qz, qw);
}
}  // namespace dai
