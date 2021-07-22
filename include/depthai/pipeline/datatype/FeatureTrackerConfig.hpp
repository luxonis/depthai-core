#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawFeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * FeatureTrackerConfig message. Carries config for feature tracking algorithm
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
     * Set feature tracking algorithm type
     * @param algorithmType Algorithm type for feature tracking
     */
    void setAlgorithmType(dai::FeatureTrackerConfigData::AlgorithmType algorithmType);

    /**
     * Set corner detector algorithm type
     * @param cornerDetector Corner detector type, HARRIS or SHI_THOMASI
     */
    void setCornerDetector(dai::FeatureTrackerConfigData::CornerDetector cornerDetector);

    /**
     * Set target number of features to detect
     * @param targetNrFeatures Number of features
     */
    void setTargetNrFeatures(std::int32_t targetNrFeatures);

    /**
     * Retrieve configuration data for FeatureTracker
     * @returns config for feature tracking algorithm
     */
    FeatureTrackerConfigData getConfigData() const;
};

}  // namespace dai
