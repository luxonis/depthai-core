#include "depthai/pipeline/datatype/AprilTagConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> AprilTagConfig::serialize() const {
    return raw;
}

AprilTagConfig::AprilTagConfig() : Buffer(std::make_shared<RawAprilTagConfig>()), cfg(*dynamic_cast<RawAprilTagConfig*>(raw.get())) {}
AprilTagConfig::AprilTagConfig(std::shared_ptr<RawAprilTagConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawAprilTagConfig*>(raw.get())) {}

void AprilTagConfig::setType(AprilTagConfigData::AprilTagType type) {
    cfg.config.type = type;
}

AprilTagConfigData AprilTagConfig::getConfigData() const {
    return cfg.config;
}

}  // namespace dai
