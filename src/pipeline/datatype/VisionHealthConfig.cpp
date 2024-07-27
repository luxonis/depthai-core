#include "depthai/pipeline/datatype/VisionHealthConfig.hpp"

namespace dai {

VisionHealthConfig::Serialized VisionHealthConfig::serialize() const {
    return {data, raw};
}

VisionHealthConfig::VisionHealthConfig() : Buffer(std::make_shared<RawVisionHealthConfig>()), cfg(*dynamic_cast<RawVisionHealthConfig*>(raw.get())) {}
VisionHealthConfig::VisionHealthConfig(std::shared_ptr<RawVisionHealthConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawVisionHealthConfig*>(raw.get())) {}

VisionHealthConfig& VisionHealthConfig::addRelativeVisionHealthConfig(VisionHealthMetricTypes metric,
                                                                      std::string oper,
                                                                      tl::optional<float> threshold,
                                                                      float sigmas) {
    cfg.relativeVisionHealthConfigs[metric] = {threshold, oper, sigmas};
    return *this;
}

VisionHealthConfig& VisionHealthConfig::addAbsoluteVisionHealthConfig(VisionHealthMetricTypes metric, std::string oper, float threshold) {
    cfg.absoluteVisionHealthConfigs[metric] = {threshold, oper};
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
