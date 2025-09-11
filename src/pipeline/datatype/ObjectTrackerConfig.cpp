#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"

namespace dai {

ObjectTrackerConfig::~ObjectTrackerConfig() = default;

void ObjectTrackerConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ObjectTrackerConfig;
}

ObjectTrackerConfig& ObjectTrackerConfig::forceRemoveID(int32_t id) {
    trackletIdsToRemove.push_back(id);
    return *this;
}

ObjectTrackerConfig& ObjectTrackerConfig::forceRemoveIDs(std::vector<int32_t> ids) {
    trackletIdsToRemove.insert(trackletIdsToRemove.end(), ids.begin(), ids.end());
    return *this;
}

}  // namespace dai
