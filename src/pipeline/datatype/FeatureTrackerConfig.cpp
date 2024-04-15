#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {

FeatureTrackerConfig& FeatureTrackerConfig::setCornerDetector(FeatureTrackerConfig::CornerDetector::Type cornerDetector) {
    this->cornerDetector.type = cornerDetector;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setCornerDetector(FeatureTrackerConfig::CornerDetector config) {
    cornerDetector = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setMotionEstimator(bool enable) {
    motionEstimator.enable = enable;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setMotionEstimator(FeatureTrackerConfig::MotionEstimator config) {
    motionEstimator = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setOpticalFlow() {
    motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    setMotionEstimator(true);
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setOpticalFlow(FeatureTrackerConfig::MotionEstimator::OpticalFlow config) {
    motionEstimator.type = FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    motionEstimator.opticalFlow = config;
    setMotionEstimator(true);
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setHwMotionEstimation() {
    motionEstimator.type = FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION;
    setMotionEstimator(true);
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setFeatureMaintainer(bool enable) {
    featureMaintainer.enable = enable;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setFeatureMaintainer(FeatureTrackerConfig::FeatureMaintainer config) {
    featureMaintainer = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setNumTargetFeatures(std::int32_t numTargetFeatures) {
    cornerDetector.numTargetFeatures = numTargetFeatures;
    return *this;
}

}  // namespace dai
