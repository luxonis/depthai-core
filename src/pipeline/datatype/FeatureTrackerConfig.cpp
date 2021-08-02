#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> FeatureTrackerConfig::serialize() const {
    return raw;
}

FeatureTrackerConfig::FeatureTrackerConfig() : Buffer(std::make_shared<RawFeatureTrackerConfig>()), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}
FeatureTrackerConfig::FeatureTrackerConfig(std::shared_ptr<RawFeatureTrackerConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}

dai::RawFeatureTrackerConfig FeatureTrackerConfig::get() const {
    return cfg;
}

void FeatureTrackerConfig::setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type cornerDetector) {
    cfg.cornerDetector.type = cornerDetector;
}

void FeatureTrackerConfig::setCornerDetector(dai::FeatureTrackerConfig::CornerDetector config) {
    cfg.cornerDetector = config;
}

void FeatureTrackerConfig::setMotionEstimator(bool enable) {
    cfg.motionEstimator.enable = enable;
}

void FeatureTrackerConfig::setMotionEstimator(dai::FeatureTrackerConfig::MotionEstimator config) {
    cfg.motionEstimator = config;
}

void FeatureTrackerConfig::setOpticalFlow() {
    cfg.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    setMotionEstimator(true);
}

void FeatureTrackerConfig::setOpticalFlow(dai::FeatureTrackerConfig::MotionEstimator::OpticalFlow config) {
    cfg.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    cfg.motionEstimator.opticalFlow = config;
    setMotionEstimator(true);
}

void FeatureTrackerConfig::setHwMotionEstimation() {
    cfg.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION;
    setMotionEstimator(true);
}

void FeatureTrackerConfig::setFeatureMaintainer(bool enable) {
    cfg.featureMaintainer.enable = enable;
}

void FeatureTrackerConfig::setFeatureMaintainer(dai::FeatureTrackerConfig::FeatureMaintainer config) {
    cfg.featureMaintainer = config;
}

void FeatureTrackerConfig::set(dai::RawFeatureTrackerConfig config) {
    cfg = config;
}

void FeatureTrackerConfig::setNumTargetFeatures(std::int32_t numTargetFeatures) {
    cfg.cornerDetector.numTargetFeatures = numTargetFeatures;
}

}  // namespace dai
