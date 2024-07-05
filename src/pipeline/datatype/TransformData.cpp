#include "depthai/pipeline/datatype/TransformData.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

namespace dai {

TransformData::TransformData(){}
TransformData::TransformData(const Transform& transform):transform(transform) {}
TransformData::TransformData(const std::array<std::array<double, 4>, 4>& data) :transform({data}) {}
TransformData::TransformData(double x, double y, double z, double qx, double qy, double qz, double qw) {
    // x,y,z,qx,qy,qz,qw to homography matrix
    double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx *= n;
    qy *= n;
    qz *= n;
    qw *= n;
    transform.matrix = {{
        {1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw, x},
        {2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw, y},
        {2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy, z},
        {0.0f, 0.0f, 0.0f, 1.0f}
    }};

    
}
TransformData::TransformData(double x, double y, double z, double roll, double pitch, double yaw) {
    // x,y,z,r,p,yw to homography matrix
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    transform.matrix = {{
        {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x},
        {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y},
        {-sp, cp*sr, cp*cr, z},
        {0.0f, 0.0f, 0.0f, 1.0f}
    }};
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
    if(fabs(p) < M_PI/2){
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
    if(tr > 0){
        double S = sqrt(tr+1.0f) * 2;
        qw = 0.25f * S;
        qx = (transform.matrix[2][1] - transform.matrix[1][2]) / S;
        qy = (transform.matrix[0][2] - transform.matrix[2][0]) / S;
        qz = (transform.matrix[1][0] - transform.matrix[0][1]) / S;
    } else if(transform.matrix[0][0] > transform.matrix[1][1] && transform.matrix[0][0] > transform.matrix[2][2]){
        double S = sqrt(1.0f + transform.matrix[0][0] - transform.matrix[1][1] - transform.matrix[2][2]) * 2; 
        qw = (transform.matrix[2][1] - transform.matrix[1][2]) / S;
        qx = 0.25f * S;
        qy = (transform.matrix[0][1] + transform.matrix[1][0]) / S;
        qz = (transform.matrix[0][2] + transform.matrix[2][0]) / S;
    } else if(transform.matrix[1][1] > transform.matrix[2][2]){
        double S = sqrt(1.0f + transform.matrix[1][1] - transform.matrix[0][0] - transform.matrix[2][2]) * 2; 
        qw = (transform.matrix[0][2] - transform.matrix[2][0]) / S;
        qx = (transform.matrix[0][1] + transform.matrix[1][0]) / S;
        qy = 0.25f * S;
        qz = (transform.matrix[1][2] + transform.matrix[2][1]) / S;
    } else {
        double S = sqrt(1.0f + transform.matrix[2][2] - transform.matrix[0][0] - transform.matrix[1][1]) * 2;
        qw = (transform.matrix[1][0] - transform.matrix[0][1]) / S;
        qx = (transform.matrix[0][2] + transform.matrix[2][0]) / S;
        qy = (transform.matrix[1][2] + transform.matrix[2][1]) / S;
        qz = 0.25f * S;
    }
    return Quaterniond(qx, qy, qz, qw);
}
}  // namespace dai
