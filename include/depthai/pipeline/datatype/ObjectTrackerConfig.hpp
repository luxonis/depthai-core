#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * ObjectTrackerConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class ObjectTrackerConfig : public Buffer {
   public:
    /**
     * Construct ObjectTrackerConfig message.
     */
    ObjectTrackerConfig() = default;
    virtual ~ObjectTrackerConfig();

    /**
     * Tracklet IDs to remove from tracking.
     * Tracklet will transition to REMOVED state.
     */
    std::vector<int32_t> trackletIdsToRemove;

    /**
     * Force remove a tracklet with specified ID.
     */
    ObjectTrackerConfig& forceRemoveID(int32_t id);

    /**
     * Force remove tracklets with specified IDs.
     */
    ObjectTrackerConfig& forceRemoveIDs(std::vector<int32_t> ids);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ObjectTrackerConfig;
    }

    DEPTHAI_SERIALIZE(ObjectTrackerConfig, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, trackletIdsToRemove);
};

}  // namespace dai
