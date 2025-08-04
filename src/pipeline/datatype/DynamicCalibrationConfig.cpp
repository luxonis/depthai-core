#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace dai {

DynamicCalibrationConfig& DynamicCalibrationConfig::setCalibrationMode(AlgorithmControl::RecalibrationMode mode) {
    algorithmControl.recalibrationMode = mode;
    return *this;
}
}  // namespace dai
