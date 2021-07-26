#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * FeatureTrackerData message. Carries position (X, Y) of tracked features and their ID.
 */
class FeatureTrackerData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawTrackedFeatures& rawdata;

   public:
    /**
     * Construct FeatureTrackerData message.
     */
    FeatureTrackerData();
    explicit FeatureTrackerData(std::shared_ptr<RawTrackedFeatures> ptr);
    virtual ~FeatureTrackerData() = default;

    std::vector<TrackedFeature>& trackedFeatures;
};

}  // namespace dai
