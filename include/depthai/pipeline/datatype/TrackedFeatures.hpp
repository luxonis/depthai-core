#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * TrackedFeatures message. Carries position (X, Y) of tracked features and their ID.
 */
class TrackedFeatures : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawTrackedFeatures& rawdata;

   public:
    /**
     * Construct TrackedFeatures message.
     */
    TrackedFeatures();
    explicit TrackedFeatures(std::shared_ptr<RawTrackedFeatures> ptr);
    virtual ~TrackedFeatures() = default;

    std::vector<TrackedFeature>& trackedFeatures;
};

}  // namespace dai
