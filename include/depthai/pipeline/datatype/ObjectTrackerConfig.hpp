#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawObjectTrackerConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * ObjectTrackerConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class ObjectTrackerConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawObjectTrackerConfig& cfg;

   public:
    /**
     * Construct ObjectTrackerConfig message.
     */
    ObjectTrackerConfig();
    explicit ObjectTrackerConfig(std::shared_ptr<RawObjectTrackerConfig> ptr);
    virtual ~ObjectTrackerConfig() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    ObjectTrackerConfig& set(dai::RawObjectTrackerConfig config);

    /**
     * Retrieve configuration data for SpatialLocationCalculator.
     * @returns config for SpatialLocationCalculator
     */
    dai::RawObjectTrackerConfig get() const;

    /**
     */
    ObjectTrackerConfig& forceRemoveID(int32_t id);

    /**
     */
    ObjectTrackerConfig& forceRemoveIDs(std::vector<int32_t> ids);
};

}  // namespace dai
