#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ImageAlignConfig::serialize() const {
    return raw;
}

ImageAlignConfig::ImageAlignConfig() : Buffer(std::make_shared<RawImageAlignConfig>()), cfg(*dynamic_cast<RawImageAlignConfig*>(raw.get())) {}
ImageAlignConfig::ImageAlignConfig(std::shared_ptr<RawImageAlignConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawImageAlignConfig*>(raw.get())) {}

dai::RawImageAlignConfig ImageAlignConfig::get() const {
    return cfg;
}

ImageAlignConfig& ImageAlignConfig::set(dai::RawImageAlignConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai