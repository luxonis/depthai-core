#include "depthai/pipeline/datatype/Keypoints.hpp"

namespace dai {

const std::vector<Point3f>& Keypoints::getKeypoints() const {
    return keypoints;
}

Keypoints& Keypoints::setKeypoints(const std::vector<Point3f>& points) {
    keypoints = points;
    return *this;
}
 
}  // namespace dai
