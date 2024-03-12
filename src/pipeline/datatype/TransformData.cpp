#include "depthai/pipeline/datatype/TransformData.hpp"


namespace dai {

TransformData::TransformData(){}
TransformData::TransformData(const Transform& transform):transform(transform) {}
TransformData::TransformData(const std::vector<std::vector<float>>& data) :transform({data}) {}
TransformData::TransformData(float x, float y, float z, float qx, float qy, float qz, float qw) {
    // x,y,z,qx,qy,qz,qw to homography matrix
    float n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx *= n;
    qy *= n;
    qz *= n;
    qw *= n;
    transform.data = {
        {1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw, x},
        {2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw, y},
        {2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy, z},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
}
TransformData::TransformData(float x, float y, float z, float roll, float pitch, float yaw) {
    // x,y,z,r,p,yw to homography matrix
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw), sy = sin(yaw);
    transform.data = {
        {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x},
        {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y},
        {-sp, cp*sr, cp*cr, z},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
}

void TransformData::getTranslation(float& x, float& y, float& z) const {
    x = transform.data[0][3];
    y = transform.data[1][3];
    z = transform.data[2][3];
}
void TransformData::getRotationEuler(float& r, float& p, float& yw) const {
    p = -asin(transform.data[2][0]);
    if(fabs(p) < M_PI/2){
        r = atan2(transform.data[2][1], transform.data[2][2]);
        yw = atan2(transform.data[1][0], transform.data[0][0]);
    } else {
        r = atan2(-transform.data[0][1], transform.data[1][1]);
        yw = 0;
    }
}
void TransformData::getQuaternion(float& qx, float& qy, float& qz, float& qw) const {
    float tr = transform.data[0][0] + transform.data[1][1] + transform.data[2][2];
    if(tr > 0){
        float S = sqrt(tr+1.0f) * 2;
        qw = 0.25f * S;
        qx = (transform.data[2][1] - transform.data[1][2]) / S;
        qy = (transform.data[0][2] - transform.data[2][0]) / S;
        qz = (transform.data[1][0] - transform.data[0][1]) / S;
    } else if(transform.data[0][0] > transform.data[1][1] && transform.data[0][0] > transform.data[2][2]){
        float S = sqrt(1.0f + transform.data[0][0] - transform.data[1][1] - transform.data[2][2]) * 2; 
        qw = (transform.data[2][1] - transform.data[1][2]) / S;
        qx = 0.25f * S;
        qy = (transform.data[0][1] + transform.data[1][0]) / S;
        qz = (transform.data[0][2] + transform.data[2][0]) / S;
    } else if(transform.data[1][1] > transform.data[2][2]){
        float S = sqrt(1.0f + transform.data[1][1] - transform.data[0][0] - transform.data[2][2]) * 2; 
        qw = (transform.data[0][2] - transform.data[2][0]) / S;
        qx = (transform.data[0][1] + transform.data[1][0]) / S;
        qy = 0.25f * S;
        qz = (transform.data[1][2] + transform.data[2][1]) / S;
    } else {
        float S = sqrt(1.0f + transform.data[2][2] - transform.data[0][0] - transform.data[1][1]) * 2;
        qw = (transform.data[1][0] - transform.data[0][1]) / S;
        qx = (transform.data[0][2] + transform.data[2][0]) / S;
        qy = (transform.data[1][2] + transform.data[2][1]) / S;
        qz = 0.25f * S;
    }
}
}  // namespace dai
