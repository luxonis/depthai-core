#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {

AprilTagConfig::Serialized AprilTagConfig::serialize() const {
    return {data, raw};
}

AprilTagConfig::AprilTagConfig() : Buffer(std::make_shared<RawAprilTagConfig>()), cfg(*dynamic_cast<RawAprilTagConfig*>(raw.get())) {}
AprilTagConfig::AprilTagConfig(std::shared_ptr<RawAprilTagConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawAprilTagConfig*>(raw.get())) {}

AprilTagConfig& AprilTagConfig::setFamily(Family family) {
    cfg.family = family;
    return *this;
}

dai::RawAprilTagConfig AprilTagConfig::get() const {
    return cfg;
}

AprilTagConfig& AprilTagConfig::set(dai::RawAprilTagConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai
