#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"

namespace dai {

ObjectTrackerConfig& ObjectTrackerConfig::forceRemoveID(int32_t id) {
    trackletIdsToRemove.push_back(id);
    return *this;
}

ObjectTrackerConfig& ObjectTrackerConfig::forceRemoveIDs(std::vector<int32_t> ids) {
    trackletIdsToRemove.insert(trackletIdsToRemove.end(), ids.begin(), ids.end());
    return *this;
}

}  // namespace dai
