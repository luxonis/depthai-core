#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * FeatureTrackerData message. Carries spatial information (X,Y,Z) and their configuration parameters
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

    std::vector<TrackedFeatures>& trackedFeatures;
};

}  // namespace dai
