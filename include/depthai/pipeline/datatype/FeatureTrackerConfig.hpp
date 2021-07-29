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
     * Set corner detector algorithm type
     * @param cornerDetector Corner detector type, HARRIS or SHI_THOMASI
     */
    void setCornerDetector(dai::FeatureTrackerConfigData::CornerDetector::AlgorithmType cornerDetector);

    /**
     * Set corner detector full configuration
     * @param config Corner detector configuration
     */
    void setCornerDetector(dai::FeatureTrackerConfigData::CornerDetector config);

    /**
     * Set optical flow full configuration
     * @param config Optical flow configuration
     */
    void setOpticalFlow(dai::FeatureTrackerConfigData::MotionEstimator::OpticalFlow config);

    /**
     * Set hardware accelerated motion estiomation using block matching.
     * Faster than optical flow (software implementation) but might not be as accurate.
     */
    void setHwMotionEstimation();

    /**
     * Set target number of features to detect
     * @param targetNumFeatures Number of features
     */
    void setTargetNumFeatures(std::int32_t targetNumFeatures);

    /**
     * Enable or disable motion estimator
     * @param enable
     */
    void setMotionEstimator(bool enable);

    /**
     * Set motion estimator full configuration
     * @param config Motion estimator configuration
     */
    void setMotionEstimator(dai::FeatureTrackerConfigData::MotionEstimator config);

    /**
     * Enable or disable feature maintainer
     * @param enable
     */
    void setFeatureMaintainer(bool enable);

    /**
     * Set feature maintainer full configuration
     * @param config feature maintainer configuration
     */
    void setFeatureMaintainer(dai::FeatureTrackerConfigData::FeatureMaintainer config);

    /**
     * Set explicit configuration
     * @param config Explicit configuration
     */
    void set(dai::FeatureTrackerConfigData config);

    /**
     * Retrieve configuration data for FeatureTracker
     * @returns config for feature tracking algorithm
     */
    FeatureTrackerConfigData get() const;
};

}  // namespace dai
