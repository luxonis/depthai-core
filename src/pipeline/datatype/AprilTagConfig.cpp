#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {

#if defined(__clang__)
AprilTagConfig::~AprilTagConfig() = default;
#endif

AprilTagConfig& AprilTagConfig::setFamily(Family family) {
    this->family = family;
    return *this;
}

}  // namespace dai
