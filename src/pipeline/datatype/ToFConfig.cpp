#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

ToFConfig::Serialized ToFConfig::serialize() const {
    return {data, raw};
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

}  // namespace dai
