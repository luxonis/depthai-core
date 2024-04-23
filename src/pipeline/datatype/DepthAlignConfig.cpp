#include "depthai/pipeline/datatype/DepthAlignConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> DepthAlignConfig::serialize() const {
    return raw;
}

DepthAlignConfig::DepthAlignConfig() : Buffer(std::make_shared<RawDepthAlignConfig>()), cfg(*dynamic_cast<RawDepthAlignConfig*>(raw.get())) {}
DepthAlignConfig::DepthAlignConfig(std::shared_ptr<RawDepthAlignConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawDepthAlignConfig*>(raw.get())) {}

dai::RawDepthAlignConfig DepthAlignConfig::get() const {
    return cfg;
}

DepthAlignConfig& DepthAlignConfig::set(dai::RawDepthAlignConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai