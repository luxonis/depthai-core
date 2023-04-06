#include "depthai/pipeline/datatype/TofCameraConfig.hpp"

namespace dai {

TofCameraConfig::Serialized TofCameraConfig::serialize() const {
    return {data, raw};
}

TofCameraConfig::TofCameraConfig() : Buffer(std::make_shared<RawTofCameraConfig>()), cfg(*dynamic_cast<RawTofCameraConfig*>(raw.get())) {}
TofCameraConfig::TofCameraConfig(std::shared_ptr<RawTofCameraConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawTofCameraConfig*>(raw.get())) {}

dai::RawTofCameraConfig TofCameraConfig::get() const {
    return cfg;
}

TofCameraConfig& TofCameraConfig::set(dai::RawTofCameraConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai
