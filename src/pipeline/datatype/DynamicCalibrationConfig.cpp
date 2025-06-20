#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace dai {

DynamicCalibrationConfig& DynamicCalibrationConfig::setCalibrationMode(AlgorithmControl::RecalibrationMode mode) {
    algorithmControl.recalibrationMode = mode;
    return *this;
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
    coverageCheckThresholds.coverageCheckThreshold = coverageThreshold;
    return *this;
}

uint8_t DynamicCalibrationConfig::getCoverageCheckThreshold() const {
    return coverageCheckThresholds.coverageCheckThreshold;
}


DynamicCalibrationConfig& DynamicCalibrationConfig::setEpipolarErrorChangeThresholds(float value) {
    calibCheckThresholds.epipolarErrorChangeThresholds = value;
    return *this;
}

float DynamicCalibrationConfig::getEpipolarErrorChangeThresholds() const {
    return calibCheckThresholds.epipolarErrorChangeThresholds;
}

DynamicCalibrationConfig::CalibCheckThresholds::RotationChangeThresholds DynamicCalibrationConfig::getRotationChangeThresholds() const {
    return calibCheckThresholds.rotationChangeThresholds;
}

DynamicCalibrationConfig& DynamicCalibrationConfig::setFlashNewCalibration(bool enable) {
    recalibrationThresholds.flashNewCalibration = enable;
    return *this;
}
}  // namespace dai
