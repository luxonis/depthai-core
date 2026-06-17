#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ObjectTrackerConfig::serialize() const {
    return raw;
}

ObjectTrackerConfig::ObjectTrackerConfig() : Buffer(std::make_shared<RawObjectTrackerConfig>()), cfg(*dynamic_cast<RawObjectTrackerConfig*>(raw.get())) {}
ObjectTrackerConfig::ObjectTrackerConfig(std::shared_ptr<RawObjectTrackerConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawObjectTrackerConfig*>(raw.get())) {}

dai::RawObjectTrackerConfig ObjectTrackerConfig::get() const {
    return cfg;
}

ObjectTrackerConfig& ObjectTrackerConfig::set(dai::RawObjectTrackerConfig config) {
    cfg = std::move(config);
    return *this;
}

ObjectTrackerConfig& ObjectTrackerConfig::forceRemoveID(int32_t id) {
    cfg.trackletIdsToRemove.push_back(id);
    return *this;
}

ObjectTrackerConfig& ObjectTrackerConfig::forceRemoveIDs(std::vector<int32_t> ids) {
    cfg.trackletIdsToRemove.insert(cfg.trackletIdsToRemove.end(), ids.begin(), ids.end());
    return *this;
}

}  // namespace dai
