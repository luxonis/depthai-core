#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ToFConfig::serialize() const {
    return raw;
}

ToFConfig::ToFConfig() : Buffer(std::make_shared<RawToFConfig>()), cfg(*dynamic_cast<RawToFConfig*>(raw.get())) {}
ToFConfig::ToFConfig(std::shared_ptr<RawToFConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawToFConfig*>(raw.get())) {}

dai::RawToFConfig ToFConfig::get() const {
    return cfg;
}

ToFConfig& ToFConfig::set(dai::RawToFConfig config) {
    cfg = config;
    return *this;
}

ToFConfig& ToFConfig::setMedianFilter(MedianFilter median) {
    cfg.median = median;
    return *this;
}

}  // namespace dai
