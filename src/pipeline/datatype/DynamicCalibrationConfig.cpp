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

}  // namespace dai
