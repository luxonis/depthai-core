#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawFeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * FeatureTrackerConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class FeatureTrackerConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawFeatureTrackerConfig& cfg;

   public:
    /**
     * Construct FeatureTrackerConfig message.
     */
    FeatureTrackerConfig();
    explicit FeatureTrackerConfig(std::shared_ptr<RawFeatureTrackerConfig> ptr);
    virtual ~FeatureTrackerConfig() = default;

    /**
     * Retrieve configuration data for FeatureTracker
     * @returns Vector of configuration parameters for ROIs (region of interests)
     */
    FeatureTrackerConfigData getConfigData() const;
};

}  // namespace dai
