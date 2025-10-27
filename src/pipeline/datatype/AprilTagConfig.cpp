#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {

AprilTagConfig::~AprilTagConfig() = default;

void AprilTagConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    (void)metadata;
    datatype = DatatypeEnum::AprilTagConfig;
};

AprilTagConfig& AprilTagConfig::setFamily(Family family) {
    this->family = family;
    return *this;
}

}  // namespace dai
