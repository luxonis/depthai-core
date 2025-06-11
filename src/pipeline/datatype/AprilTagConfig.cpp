#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {
AprilTagConfig& AprilTagConfig::setFamily(Family family) {
    this->family = family;
    return *this;
}

}  // namespace dai
