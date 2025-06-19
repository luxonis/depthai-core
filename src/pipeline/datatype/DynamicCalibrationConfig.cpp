#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace dai {

DynamicCalibrationConfig& DynamicCalibrationConfig::setCalibrationMode(AlgorithmControl::RecalibrationMode mode) {
    algorithmControl.recalibrationMode = mode;
    return *this;
}

DynamicCalibrationConfig& DynamicCalibrationConfig::setTimePeriod(uint8_t time, AlgorithmControl::TimeUnit timeUnit) {
    algorithmControl.timeUnit = timeUnit;
    algorithmControl.periodTime = time;
    return *this;
}

uint8_t DynamicCalibrationConfig::getTimePeriod() const {
    return algorithmControl.periodTime;
}

DynamicCalibrationConfig& DynamicCalibrationConfig::setEnableCoverageCheck(bool enable) {
    algorithmControl.enableCoverageCheck = enable;
    return *this;
}

bool DynamicCalibrationConfig::getEnableCoverageCheck()  const {
    return algorithmControl.enableCoverageCheck;
}

DynamicCalibrationConfig& DynamicCalibrationConfig::setMaximumCalibCheckFrames(uint8_t maxFramesPerCheck) {
    algorithmControl.maximumCalibCheckFrames = maxFramesPerCheck;
    return *this;
}

DynamicCalibrationConfig& DynamicCalibrationConfig::setMaximumRecalibrationFrames(uint8_t maxFramesPerRecalib) {
    algorithmControl.maximumRecalibrationFrames = maxFramesPerRecalib;
    return *this;
}

uint8_t DynamicCalibrationConfig::getMaximumCalibCheckFrames() const {
    return algorithmControl.maximumCalibCheckFrames;
}

uint8_t DynamicCalibrationConfig::getMaximumRecalibrationFrames() const {
    return algorithmControl.maximumRecalibrationFrames;
}


DynamicCalibrationConfig& DynamicCalibrationConfig::setCoverageCheckThreshold(uint8_t coverageThreshold) {
    coverageCheckMetrics.coverageCheckThreshold = coverageThreshold;
    return *this;
}

uint8_t DynamicCalibrationConfig::getCoverageCheckThreshold() const {
    return coverageCheckMetrics.coverageCheckThreshold;
}


DynamicCalibrationConfig& DynamicCalibrationConfig::setEpipolarErrorChangeThresholds(float value) {
    calibCheckMetrics.epipolarErrorChangeThresholds = value;
    return *this;
}

float DynamicCalibrationConfig::getEpipolarErrorChangeThresholds() const {
    return calibCheckMetrics.epipolarErrorChangeThresholds;
}

DynamicCalibrationConfig::CalibCheckMetrics::RotationChangeThresholds DynamicCalibrationConfig::getRotationChangeThresholds() const {
    return calibCheckMetrics.rotationChangeThresholds;
}

DynamicCalibrationConfig& DynamicCalibrationConfig::setFlashNewCalibration(bool enable) {
    recalibrationMetrics.flashNewCalibration = enable;
    return *this;
}
}  // namespace dai
