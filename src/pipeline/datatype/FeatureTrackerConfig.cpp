#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> FeatureTrackerConfig::serialize() const {
    return raw;
}

FeatureTrackerConfig::FeatureTrackerConfig() : Buffer(std::make_shared<RawFeatureTrackerConfig>()), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}
FeatureTrackerConfig::FeatureTrackerConfig(std::shared_ptr<RawFeatureTrackerConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}

FeatureTrackerConfigData FeatureTrackerConfig::getConfigData() const {
    return cfg.config;
}

}  // namespace dai
