#pragma once

namespace dai {

enum class AutoCalibrationExecutionStatus {
    IDLE,
    RUNNING,
    VALIDATING_INPUT,
    CALIBRATING,
    TIMEOUT,
    INVALID_INPUT,
    SUCCEEDED,
    FAILED,
    STOPPED,
};

struct AutoCalibrationStatus {
    AutoCalibrationExecutionStatus status = AutoCalibrationExecutionStatus::IDLE;
    double dataConfidence = 0.0;
    double calibrationConfidence = 0.0;
    bool calibrationResult = false;
};

}  // namespace dai
