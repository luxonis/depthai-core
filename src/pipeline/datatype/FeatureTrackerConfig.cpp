#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> FeatureTrackerConfig::serialize() const {
    return raw;
}

FeatureTrackerConfig::FeatureTrackerConfig() : Buffer(std::make_shared<RawFeatureTrackerConfig>()), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}
FeatureTrackerConfig::FeatureTrackerConfig(std::shared_ptr<RawFeatureTrackerConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}

dai::FeatureTrackerConfigData FeatureTrackerConfig::get() const {
    return cfg.config;
}

void FeatureTrackerConfig::setCornerDetector(dai::FeatureTrackerConfigData::CornerDetector::Type cornerDetector) {
    cfg.config.cornerDetector.type = cornerDetector;
}

void FeatureTrackerConfig::setCornerDetector(dai::FeatureTrackerConfigData::CornerDetector config) {
    cfg.config.cornerDetector = config;
}

void FeatureTrackerConfig::setMotionEstimator(bool enable) {
    cfg.config.motionEstimator.enable = enable;
}

void FeatureTrackerConfig::setMotionEstimator(dai::FeatureTrackerConfigData::MotionEstimator config) {
    cfg.config.motionEstimator = config;
}

void FeatureTrackerConfig::setOpticalFlow(dai::FeatureTrackerConfigData::MotionEstimator::OpticalFlow config) {
    cfg.config.motionEstimator.type = dai::FeatureTrackerConfigData::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    cfg.config.motionEstimator.opticalFlow = config;
    setMotionEstimator(true);
}

void FeatureTrackerConfig::setHwMotionEstimation() {
    cfg.config.motionEstimator.type = dai::FeatureTrackerConfigData::MotionEstimator::Type::HW_MOTION_ESTIMATION;
    setMotionEstimator(true);
}

void FeatureTrackerConfig::setFeatureMaintainer(bool enable) {
    cfg.config.featureMaintainer.enable = enable;
}

void FeatureTrackerConfig::setFeatureMaintainer(dai::FeatureTrackerConfigData::FeatureMaintainer config) {
    cfg.config.featureMaintainer = config;
}

void FeatureTrackerConfig::set(dai::FeatureTrackerConfigData config) {
    cfg.config = config;
}

void FeatureTrackerConfig::setNumTargetFeatures(std::int32_t numTargetFeatures) {
    cfg.config.cornerDetector.numTargetFeatures = numTargetFeatures;
}

}  // namespace dai
