#include "depthai/pipeline/datatype/VisionHealthConfig.hpp"

namespace dai {

VisionHealthConfig::Serialized VisionHealthConfig::serialize() const {
    return {data, raw};
}

VisionHealthConfig::VisionHealthConfig() : Buffer(std::make_shared<RawVisionHealthConfig>()), cfg(*dynamic_cast<RawVisionHealthConfig*>(raw.get())) {}
VisionHealthConfig::VisionHealthConfig(std::shared_ptr<RawVisionHealthConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawVisionHealthConfig*>(raw.get())) {}

VisionHealthConfig& VisionHealthConfig::addVisionHealthConfig(VisionHealthMetricTypes metric) {
    cfg.visionHealthConfigs.push_back(metric);
    return *this;
}

dai::RawVisionHealthConfig VisionHealthConfig::get() const {
    return cfg;
}

VisionHealthConfig& VisionHealthConfig::set(dai::RawVisionHealthConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai
